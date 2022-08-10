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
-- | Create <https://www.ros.org/ Robot Operating System> (ROS) applications
-- that subscribe to obtain data and call Copilot when new values arrive.
--
-- It is the user's responsibility to modify the generated Copilot/C/C++ code
-- to deal with the monitors they'd like to implement, and the data they must
-- manipulate.

{- HLINT ignore "Functor law" -}
module Command.ROSApp
    ( rosApp
    , ErrorCode
    )
  where

-- External imports
import qualified Control.Exception as E
import           Data.List         ( find, intersperse )
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
       -> FilePath       -- ^ File containing a list of handlers used in the
                         --   Copilot specification. The handlers are assumed
                         --   to receive no arguments.
       -> IO (Result ErrorCode)
rosApp targetDir varNameFile varDBFile handlersFile = do

  -- We first try to open the files we need to fill in details in the ROS app
  -- template.
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

          -- The handler list is mandatory. This check fails if the filename
          -- provided does not exist or if the file cannot be opened. The
          -- condition on the result also checks that the list of tiggers in
          -- the file is not empty (otherwise, we do not know when to call
          -- Copilot).
          handlerNamesE <- E.try $ lines <$> readFile handlersFile

          case handlerNamesE of
            Left e         -> return $ cannotOpenHandlersFile handlersFile e
            Right []       -> return $ cannotEmptyHandlerList handlersFile
            Right monitors -> do

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
                          ( vars : oVars
                          , ids : oIds
                          , infos : oInfos
                          , datas : oDatas
                          )

                -- This is a Data.List.unzip4
                let (vars, ids, infos, datas) =
                      foldr f ([], [], [], []) varNames

                let rosFileName =
                      targetDir </> "src" </> "copilot_member_function.cpp"
                    rosFileContents =
                      unlines $
                        fileContents varNames vars ids infos datas monitors

                writeFile rosFileName rosFileContents

                let rosFileName =
                      targetDir </> "src" </> "copilot_logger.cpp"
                    rosFileContents =
                      unlines $
                        logger varNames vars ids infos datas monitors

                writeFile rosFileName rosFileContents

                return Success

-- | Predefined list of ROS variables that are known to Ogma
knownVars :: [(String, String, String, String)]
knownVars = []

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

    -- Convert a DB row into Variable info needed to generate the ROS file
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

-- | Return the contents of the main ROS application.
fileContents :: [String]     -- Variables
             -> [VarDecl]
             -> [MsgInfoId]
             -> [MsgInfo]
             -> [MsgData]
             -> [String]     -- Monitors
             -> [String]
fileContents varNames variables msgIds msgNames msgDatas monitors =
    rosFileContents

  where

    rosFileContents =
      [ "#include <functional>"
      , "#include <memory>"
      , ""
      , "#include \"rclcpp/rclcpp.hpp\""
      , ""
      , typeIncludes
      , copilotIncludes
      , "using std::placeholders::_1;"
      , ""
      , variablesS
      , "class CopilotRV : public rclcpp::Node {"
      , "  public:"
      , "    CopilotRV() : Node(\"copilotrv\") {"
      , msgSubscriptionS
      , msgPublisherS
      , "    }"
      , ""
      , msgHandlerInClassS
      , "    // Needed so we can report messages to the log."
      , "    static CopilotRV& getInstance() {"
      , "      static CopilotRV instance;"
      , "      return instance;"
      , "    }"
      , ""
      , "  private:"
      , msgCallbacks
      , msgSubscriptionDeclrs
      , msgPublisherDeclrs
      , "};"
      , ""
      , msgHandlerGlobalS
      , "int main(int argc, char* argv[]) {"
      , "  rclcpp::init(argc, argv);"
      , "  rclcpp::spin(std::make_shared<CopilotRV>());"
      , "  rclcpp::shutdown();"
      , "  return 0;"
      , "}"
      ]

    msgHandlerInClassS = unlines
                       $ concat
                       $ intersperse [""]
                       $ map msgHandlerInClass monitors
    msgHandlerInClass monitor =
        [ "    // Report monitor violations to the log and publish them."
        , "    void " ++ handlerName ++ "() {"
        , "      // RCLCPP_INFO(this->get_logger(), " ++ show handlerName ++ ");"
        , "      auto output = " ++ ty ++ "();"
        , "      " ++ publisher ++ "->publish(output);"
        , "    }"
        ]
      where
        handlerName :: String
        handlerName = monitor

        ty = "std_msgs::msg::Empty"

        publisher = monitor ++ "_publisher_"

    typeIncludes = unlines
      [ "#include \"std_msgs/msg/bool.hpp\""
      , "#include \"std_msgs/msg/empty.hpp\""
      , "#include \"std_msgs/msg/u_int8.hpp\""
      , "#include \"std_msgs/msg/u_int16.hpp\""
      , "#include \"std_msgs/msg/u_int32.hpp\""
      , "#include \"std_msgs/msg/u_int64.hpp\""
      , "#include \"std_msgs/msg/int8.hpp\""
      , "#include \"std_msgs/msg/int16.hpp\""
      , "#include \"std_msgs/msg/int32.hpp\""
      , "#include \"std_msgs/msg/int64.hpp\""
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

    msgSubscriptionS     = unlines
                         $ concat
                         $ intersperse [""]
                         $ map toMsgSubscription (zip variables msgIds)
    toMsgSubscription (nm, msgId) =
        [ "      " ++ subscription
                   ++ " = this->create_subscription<" ++ ty ++ ">("
        , "        \"" ++ topic ++ "\", " ++ show unknownVar ++ ","
        , "        std::bind(&CopilotRV::" ++ callback ++ ", this, _1));"
        ]
      where
        ty           = varDeclMsgType nm
        topic        = msgId -- varDeclName nm
        subscription = varDeclName nm ++ "_subscription_"
        callback     = varDeclName nm ++ "_callback"

        unknownVar   :: Int
        unknownVar   = 10

    msgPublisherS = unlines
                  $ concat
                  $ intersperse [""]
                  $ map toMsgPublisher monitors

    toMsgPublisher nm =
        [ "      " ++ publisher
                   ++ " = this->create_publisher<" ++ ty ++ ">("
        , "        \"" ++ topic ++ "\", " ++ show unknownVar ++ ");"
        ]
      where
        ty        = "std_msgs::msg::Empty"
        publisher = nm ++ "_publisher_"
        topic     = "copilot/" ++ nm

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

    msgCallbacks = unlines
                 $ concat
                 $ intersperse [""]
                 $ map toCallback variables
    toCallback varDecl =
        [ "    void " ++ callback
                      ++ "(const " ++ ty ++ "::SharedPtr msg) const {"
        , "      " ++ variable ++ " = msg->data;"
        , "      step();"
        , "    }"
        ]
      where
        ty = varDeclMsgType varDecl
        variable = varDeclName varDecl
        callback = variable ++ "_callback"

    msgHandlerGlobalS = unlines
                      $ concat
                      $ intersperse [""]
                      $ map msgHandlerGlobal monitors
    msgHandlerGlobal monitor =
        [ "// Pass monitor violations to the actual class, which has ways to"
        , "// communicate with other applications."
        , "void " ++ handlerName ++ "() {"
        , "  CopilotRV::getInstance()." ++ handlerName ++ "();"
        , "}"
        ]
      where
        handlerName = monitor

    msgSubscriptionDeclrs :: String
    msgSubscriptionDeclrs = unlines
                          $ concat
                          $ intersperse [""]
                          $ map toSubscriptionDecl variables
    toSubscriptionDecl nm =
        [ "    rclcpp::Subscription<" ++ ty ++ ">::SharedPtr "
            ++ subscription ++ ";"
        ]
      where
        ty           = varDeclMsgType nm
        subscription = varDeclName nm ++ "_subscription_"

    msgPublisherDeclrs :: String
    msgPublisherDeclrs = unlines
                       $ concat
                       $ intersperse [""]
                       $ map toPublisherDecl monitors
    toPublisherDecl nm =
        [ "    rclcpp::Publisher<" ++ ty ++ ">::SharedPtr "
            ++ publisher ++ ";"
        ]
      where
        ty        = "std_msgs::msg::Empty"
        publisher = nm ++ "_publisher_"

-- | Return the contents of the logger ROS application.
logger :: [String]     -- Variables
       -> [VarDecl]
       -> [MsgInfoId]
       -> [MsgInfo]
       -> [MsgData]
       -> [String]     -- Monitors
       -> [String]
logger varNames variables msgIds msgNames msgDatas monitors =
    rosFileContents

  where

    rosFileContents =
      [ "#include <functional>"
      , "#include <memory>"
      , ""
      , "#include \"rclcpp/rclcpp.hpp\""
      , ""
      , typeIncludes
      , "using std::placeholders::_1;"
      , ""
      , "class CopilotLogger : public rclcpp::Node {"
      , "  public:"
      , "    CopilotLogger() : Node(\"copilotlogger\") {"
      , msgSubscriptionS
      , "    }"
      , ""
      , "  private:"
      , msgCallbacks
      , msgSubscriptionDeclrs
      , "};"
      , ""
      , "int main(int argc, char* argv[]) {"
      , "  rclcpp::init(argc, argv);"
      , "  rclcpp::spin(std::make_shared<CopilotLogger>());"
      , "  rclcpp::shutdown();"
      , "  return 0;"
      , "}"
      ]

    typeIncludes = unlines
      [ "#include \"std_msgs/msg/empty.hpp\""
      ]

    msgSubscriptionS     = unlines
                         $ concat
                         $ intersperse [""]
                         $ map toMsgSubscription monitors
    toMsgSubscription nm =
        [ "      " ++ subscription
                   ++ " = this->create_subscription<" ++ ty ++ ">("
        , "        \"" ++ topic ++ "\", " ++ show unknownVar ++ ","
        , "        std::bind(&CopilotLogger::" ++ callback ++ ", this, _1));"
        ]
      where
        ty           = "std_msgs::msg::Empty"
        topic        = "copilot/" ++ nm
        subscription = nm ++ "_subscription_"
        callback     = nm ++ "_callback"

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

    msgCallbacks = unlines
                 $ concat
                 $ intersperse [""]
                 $ map toCallback monitors
    toCallback varDecl =
        [ "    void " ++ callback
                      ++ "(const " ++ ty ++ "::SharedPtr msg) const {"
        , "      RCLCPP_INFO(this->get_logger(), \"Copilot monitor violation: " ++ varDecl ++ "\");"
        , "    }"
        ]
      where
        ty = "std_msgs::msg::Empty"
        callback = varDecl ++ "_callback"

    msgSubscriptionDeclrs :: String
    msgSubscriptionDeclrs = unlines
                          $ concat
                          $ intersperse [""]
                          $ map toSubscriptionDecl monitors
    toSubscriptionDecl nm =
        [ "    rclcpp::Subscription<" ++ ty ++ ">::SharedPtr "
            ++ subscription ++ ";"
        ]
      where
        ty           = "std_msgs::msg::Empty"
        subscription = nm ++ "_subscription_"

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

-- | Exception handler to deal with the case in which the handlers file
-- provided by the user cannot be opened.
cannotOpenHandlersFile :: FilePath -> E.SomeException -> Result ErrorCode
cannotOpenHandlersFile file _e =
    Error ecCannotOpenHandlersFile  msg (LocationFile file)
  where
    msg =
      "cannot open handler list file " ++ file

-- | Exception handler to deal with the case of the handler file provided
-- containing an empty list.
cannotEmptyHandlerList :: FilePath -> Result ErrorCode
cannotEmptyHandlerList file =
    Error ecCannotEmptyHandlerList msg (LocationFile file)
  where
    msg =
      "Handler list in file " ++ file ++ " is empty"

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

-- | Error: the handlers file provided by the user cannot be opened.
ecCannotOpenHandlersFile :: ErrorCode
ecCannotOpenHandlersFile = 1

-- | Error: the handlers file provided contains an empty list.
ecCannotEmptyHandlerList :: ErrorCode
ecCannotEmptyHandlerList = 1

-- | Error: the files cannot be copied/generated due lack of space or
-- permissions or some I/O error.
ecCannotCopyTemplate :: ErrorCode
ecCannotCopyTemplate = 1
