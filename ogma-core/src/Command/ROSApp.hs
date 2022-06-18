-- Copyright 2020 United States Government as represented by the Administrator
-- of the National Aeronautics and Space Administration. All Rights Reserved.
--
-- Disclaimers
--
-- No Warranty: THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY
-- OF ANY KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
-- LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
-- SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
-- PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT THE
-- SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT DOCUMENTATION, IF
-- PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE. THIS AGREEMENT DOES NOT, IN
-- ANY MANNER, CONSTITUTE AN ENDORSEMENT BY GOVERNMENT AGENCY OR ANY PRIOR
-- RECIPIENT OF ANY RESULTS, RESULTING DESIGNS, HARDWARE, SOFTWARE PRODUCTS OR
-- ANY OTHER APPLICATIONS RESULTING FROM USE OF THE SUBJECT SOFTWARE. FURTHER,
-- GOVERNMENT AGENCY DISCLAIMS ALL WARRANTIES AND LIABILITIES REGARDING
-- THIRD-PARTY SOFTWARE, IF PRESENT IN THE ORIGINAL SOFTWARE, AND DISTRIBUTES
-- IT "AS IS."
--
-- Waiver and Indemnity: RECIPIENT AGREES TO WAIVE ANY AND ALL CLAIMS AGAINST
-- THE UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS
-- ANY PRIOR RECIPIENT. IF RECIPIENT'S USE OF THE SUBJECT SOFTWARE RESULTS IN
-- ANY LIABILITIES, DEMANDS, DAMAGES, EXPENSES OR LOSSES ARISING FROM SUCH USE,
-- INCLUDING ANY DAMAGES FROM PRODUCTS BASED ON, OR RESULTING FROM, RECIPIENT'S
-- USE OF THE SUBJECT SOFTWARE, RECIPIENT SHALL INDEMNIFY AND HOLD HARMLESS THE
-- UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY
-- PRIOR RECIPIENT, TO THE EXTENT PERMITTED BY LAW. RECIPIENT'S SOLE REMEDY
-- FOR ANY SUCH MATTER SHALL BE THE IMMEDIATE, UNILATERAL TERMINATION OF THIS
-- AGREEMENT.
--
-- | Create <https://ros.gsfc.nasa.gov/ NASA Core Flight System> (CFS)
-- applications that subscribe to the communication bus and call Copilot when
-- new messages arrive.
--
-- The applications are created ready to be extracted in the application
-- directory in CFS, and they subscribe to a generic monitor. It is the user's
-- responsibility to modify the generated Copilot and C code to deal with the
-- monitors they'd like to implement, and the data they must manipulate.

{- HLINT ignore "Functor law" -}
module Command.ROSApp
    ( rosApp
    , ErrorCode
    )
  where

-- External imports
import qualified Control.Exception as E
import           Data.List         ( find )
import           System.FilePath   ( (</>) )

-- External imports: auxiliary
import System.Directory.Extra ( copyDirectoryRecursive )

-- Internal imports: auxiliary
import Command.Result ( Result (..) )
import Data.Location  ( Location (..) )

-- Internal imports
import Paths_ogma_core ( getDataDir )

-- | Generate a new ROS application connected to Copilot.
rosApp :: FilePath       -- ^ Target directory where the application
                         --   should be created.
       -> FilePath       -- ^ File containing a list of variables to make
                         --   available to Copilot.
       -> Maybe FilePath -- ^ File containing a list of known variables
                         --   with their types and the message IDs they
                         --   can be obtained from.
       -> IO (Result ErrorCode)
rosApp targetDir varNameFile varDBFile = do

  let monitors :: [String]
      monitors = [ "func_on"
                 , "func_off"
                 ]

  -- We first try to open the two files we need to fill in details in the CFS
  -- app template.
  --
  -- The variable DB is optional, so this check only fails if the filename
  -- provided does not exist or if the file cannot be opened or parsed (wrong
  -- format).
  varDBE <- E.try $
                case varDBFile of
                  Nothing -> return knownVars
                  Just fn -> fmap read <$> lines <$> readFile fn

  case varDBE of
    Left  e     -> return $ cannotOpenDB varDBFile e
    Right varDB -> do

      -- The variable list is mandatory. This check fails if the filename
      -- provided does not exist or if the file cannot be opened. The condition
      -- on the result also checks that the list of variables in the file is
      -- not empty (otherwise, we do not know when to call Copilot).
      varNamesE <- E.try $ lines <$> readFile varNameFile

      case varNamesE of
        Left e         -> return $ cannotOpenVarFile varNameFile e
        Right []       -> return $ cannotEmptyVarList varNameFile
        Right varNames -> do

          -- Obtain template dir
          dataDir <- getDataDir
          let templateDir = dataDir </> "templates" </> "ros"

          E.handle (return . cannotCopyTemplate) $ do
            -- Expand template
            copyDirectoryRecursive templateDir targetDir

            let f n o@(oVars, oIds, oInfos, oDatas) =
                  case variableMap varDB n of
                    Nothing -> o
                    Just (vars, ids, infos, datas) ->
                      (vars : oVars, ids : oIds, infos : oInfos, datas : oDatas)

            -- This is a Data.List.unzip4
            let (vars, ids, infos, datas) = foldr f ([], [], [], []) varNames

            let rosFileName = targetDir </> "copilot" </> "src" </> "copilot_member_function.cpp"
                rosFileContents = unlines $ fileContents varNames vars ids infos datas monitors

            writeFile rosFileName rosFileContents
            return Success

-- | Predefined list of Icarous variables that are known to Ogma
knownVars :: [(String, String, String, String)]
knownVars =
  [ ("position", "position_t", "ICAROUS_POSITION_MID", "IcarousPosition") ]

-- | Return the variable information needed to generate declarations
-- and subscriptions for a given variable name and variable database.
variableMap :: [(String, String, String, String)]
            -> String
            -> Maybe (VarDecl, MsgInfoId, MsgInfo, MsgData)
variableMap varDB varName =
    csvToVarMap <$> find (sameName varName) varDB

  where

    -- True if the given variable and db entry have the same name
    sameName :: String
             -> (String, String, String, String)
             -> Bool
    sameName n (vn, _, _, _) = n == vn

    -- Convert a DB row into Variable info needed to generate the CFS file
    csvToVarMap :: (String, String, String, String)
                -> (VarDecl, String, MsgInfo, MsgData)
    csvToVarMap (nm, ty, mid, mn) =
      (VarDecl nm ty, mid, MsgInfo mid mn, MsgData mn nm ty)

-- | The declaration of a variable in C, with a given type and name.
data VarDecl = VarDecl
  { varDeclName :: String
  , varDeclType :: String
  }

-- | The message ID to subscribe to.
type MsgInfoId = String

-- | A message ID to subscribe to and the name associated to it. The name is
-- used to generate a suitable name for the message handler.
data MsgInfo = MsgInfo
  { msgInfoId   :: MsgInfoId
  , msgInfoDesc :: String
  }

-- | Information on the data provided by a message with a given description,
-- and the type of the data it carries.
data MsgData = MsgData
  { msgDataDesc    :: String
  , msgDataVarName :: String
  , msgDataVarType :: String
  }

-- | Return the contents of the main CFS application.
fileContents :: [String]     -- Variables
             -> [VarDecl]
             -> [MsgInfoId]
             -> [MsgInfo]
             -> [MsgData]
             -> [String]     -- Monitors
             -> [String]
fileContents varNames variables msgIds msgNames msgDatas monitors = rosFileContents
  where

    rosFileContents =
      [ "#include <functional>"
      , "#include <memory>"
      , ""
      , "#include \"rclcpp/rclcpp.hpp\""
      , ""
      , typeIncludes
      , copilotIncludes
      , ""
      , "using std::placeholders::_1;"
      , ""
      , variablesS
      , ""
      , "class CopilotRVSubscriber : public rclcpp::Node {"
      , " public:"
      , "  CopilotRVSubscriber() : Node(\"minimal_subscriber\") {"
      , msgSubscriptionS
      , "  }"
      , ""
      , msgHandlerInClassS
      , ""
      , "  // Needed so we can report messages to the log."
      , "  static CopilotRVSubscriber& getInstance() {"
      , "    static CopilotRVSubscriber instance;"
      , "    return instance;"
      , "  }"
      , ""
      , " private:"
      , msgCallbacks
      , msgSubscriptionDeclrs
      , "};"
      , ""
      , msgHandlerGlobalS
      , ""
      , "int main(int argc, char* argv[]) {"
      , "  rclcpp::init(argc, argv);"
      , "  rclcpp::spin(std::make_shared<CopilotRVSubscriber>());"
      , "  rclcpp::shutdown();"
      , "  return 0;"
      , "}"
      ]

    msgHandlerInClassS = unlines $ concatMap msgHandlerInClass monitors
    msgHandlerInClass monitor =
        [ "  // Report monitor violations to the log."
        , "  void " ++ handlerName ++ "() {"
        , "    RCLCPP_INFO(this->get_logger(), " ++ show handlerName ++ ");"
        , "  }"
        ]
      where
        handlerName :: String
        handlerName = monitor
      -- , "  // Report monitor violations to the log."
      -- , "  void func_on(float temp) {"
      -- , "    RCLCPP_INFO(this->get_logger(), \"On: %f\", temp);"
      -- , "  }"
      -- , ""
      -- , "  // Report monitor violations to the log."
      -- , "  void func_off(float temp) {"
      -- , "    RCLCPP_INFO(this->get_logger(), \"Off: %f\", temp);"
      -- , "  }"

    typeIncludes = unlines
      [ "#include \"std_msgs/msg/u_int8.hpp\""
      , "#include <cstdint>"
      ]

    copilotIncludes = unlines
      [ "#include \"monitor.h\""
      , "#include \"monitor.c\""
      ]

    variablesS = unlines $ map toVarDecl variables
    toVarDecl varDecl =
        varDeclType' varDecl ++ " " ++ varDeclName varDecl ++ ";"
      where
        varDeclType' varDecl = case varDeclType varDecl of
          "uint8_t"  -> "std::uint8_t"
          "uint16_t" -> "std::uint16_t"
          "uint32_t" -> "std::uint32_t"
          "uint64_t" -> "std::uint64_t"
          "int8_t"   -> "std::int8_t"
          "int16_t"  -> "std::int16_t"
          "int32_t"  -> "std::int32_t"
          "int64_t"  -> "std::int64_t"
          def        -> def

    msgSubscriptionS     = unlines $ concatMap toMsgSubscription variables
    toMsgSubscription nm =
      [ "    " ++ subscription ++ " = this->create_subscription<" ++ ty ++ ">("
      , "      \"" ++ topic ++ "\", " ++ show unknownVar ++ ","
      , "      std::bind(&CopilotRVSubscriber::" ++ callback ++ ", this, _1));"
      ]
        -- subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
        --   "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      where
        ty           = varDeclMsgType nm                  -- std_msgs::msg::UInt8
        topic        = varDeclName nm                     -- temperature
        subscription = varDeclName nm ++ "_subscription_" -- temperature_subscription_
        callback     = varDeclName nm ++ "_callback"      -- temperature_callback

        unknownVar   :: Int
        unknownVar   = 10

    varDeclMsgType varDecl = case varDeclType varDecl of
      "uint8_t"  -> "std_msgs::msg::UInt8"
      "uint16_t" -> "std_msgs::msg::UInt16"
      "uint32_t" -> "std_msgs::msg::UInt32"
      "uint64_t" -> "std_msgs::msg::UInt64"
      "int8_t"   -> "std_msgs::msg::Int8"
      "int16_t"  -> "std_msgs::msg::Int16"
      "int32_t"  -> "std_msgs::msg::Int32"
      "int64_t"  -> "std_msgs::msg::Int64"
      def        -> def

    msgCallbacks = unlines $ map toCallback variables
    toCallback varDecl = unlines
      [ "  void " ++ callback ++ "(const " ++ ty ++ "::SharedPtr msg) const {"
      , "    " ++ variable ++ "msg->data;"
      , "    step();"
      , "  }"
      ]
      where
        ty = varDeclMsgType varDecl
        variable = varDeclName varDecl
        callback = variable ++ "_callback"

      -- , "  void temperature_callback(const std_msgs::msg::UInt8::SharedPtr msg) const {"
      -- , "    RCLCPP_INFO(this->get_logger(), \"I heard: '%d'\", msg->data);"
      -- , "    temperature += msg->data;"
      -- , "    RCLCPP_INFO(this->get_logger(), \"Executing Copilot monitors\");"
      -- , "    step();"
      -- , "  }"

    msgHandlerGlobalS = unlines $ concatMap msgHandlerGlobal monitors
    msgHandlerGlobal monitor =
      [ "// Pass monitor violations to the actual class, which has ways to communicate"
      , "// with other applications."
      , "  void " ++ handlerName ++ "() {"
      , "    CopilotRVSubscriber::getInstance()." ++ handlerName ++ "();"
      , "  }"
      ]
      where
        handlerName = monitor
      -- , "// Pass monitor violations to the actual class, which has ways to communicate"
      -- , "// with other applications."
      -- , "void heaton(float temp) {"
      -- , "  CopilotRVSubscriber::getInstance().func_on(temp);"
      -- , "}"
      -- , ""
      -- , "void heatoff(float temp) {"
      -- , "  CopilotRVSubscriber::getInstance().func_off(temp);"
      -- , "}"

    msgSubscriptionDeclrs :: String
    msgSubscriptionDeclrs = unlines $ concatMap toSubscriptionDecl variables
    toSubscriptionDecl nm =
        [ "    rclcpp::Subscription<" ++ ty ++ ">::SharedPtr " ++ subscription ++ ";" ]
      where
        ty           = varDeclMsgType nm -- std_msgs::msg::UInt8
        subscription = varDeclName nm ++ "_subscription_" -- temperature_subscription_

-- * Exception handlers

-- | Exception handler to deal with the case in which the variable DB cannot be
-- opened.
cannotOpenDB :: Maybe FilePath -> E.SomeException -> Result ErrorCode
cannotOpenDB Nothing _e =
    Error ecCannotOpenDBCritical msg LocationNothing
  where
    msg =
      "cannotOpenDB: this is a bug. Please notify the developers"
cannotOpenDB (Just file) _e =
    Error ecCannotOpenDBUser msg (LocationFile file)
  where
    msg =
      "cannot open variable DB file " ++ file

-- | Exception handler to deal with the case in which the variable file
-- provided by the user cannot be opened.
cannotOpenVarFile :: FilePath -> E.SomeException -> Result ErrorCode
cannotOpenVarFile file _e =
    Error ecCannotOpenVarFile  msg (LocationFile file)
  where
    msg =
      "cannot open variable list file " ++ file

-- | Exception handler to deal with the case of the variable file provided
-- containing an empty list.
cannotEmptyVarList :: FilePath -> Result ErrorCode
cannotEmptyVarList file =
    Error ecCannotEmptyVarList msg (LocationFile file)
  where
    msg =
      "variable list in file " ++ file ++ " is empty"

-- | Exception handler to deal with the case of files that cannot be
-- copied/generated due lack of space or permissions or some I/O error.
cannotCopyTemplate :: E.SomeException -> Result ErrorCode
cannotCopyTemplate _e =
    Error ecCannotCopyTemplate msg LocationNothing
  where
    msg =
      "ROS app generation failed during copy/write operation. Check that"
      ++ " there's free space in the disk and that you have the necessary"
      ++ " permissions to write in the destination directory."

-- * Error codes

-- | Encoding of reasons why the command can fail.
--
-- The error codes used are 1 for user error, and 2 for internal bug.
type ErrorCode = Int

-- | Internal error: Variable DB cannot be opened.
ecCannotOpenDBCritical :: ErrorCode
ecCannotOpenDBCritical = 2

-- | Error: the variable DB provided by the user cannot be opened.
ecCannotOpenDBUser :: ErrorCode
ecCannotOpenDBUser = 1

-- | Error: the variable file provided by the user cannot be opened.
ecCannotOpenVarFile :: ErrorCode
ecCannotOpenVarFile = 1

-- | Error: the variable file provided contains an empty list.
ecCannotEmptyVarList :: ErrorCode
ecCannotEmptyVarList = 1

-- | Error: the files cannot be copied/generated due lack of space or
-- permissions or some I/O error.
ecCannotCopyTemplate :: ErrorCode
ecCannotCopyTemplate = 1
