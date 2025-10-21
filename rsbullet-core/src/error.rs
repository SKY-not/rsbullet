use std::ffi::NulError;
use std::fmt::{Display, Formatter};
use std::io;

use robot_behavior::{PhysicsEngineException, RendererException, RobotException};

/// Represents failures that can occur while interacting with the Bullet physics server.
#[derive(Debug)]
pub enum BulletError {
    /// Attempted to connect but Bullet returned a null pointer.
    NullPointer(&'static str),
    /// Bullet reported that the physics server is not responsive.
    ServerUnavailable(&'static str),
    /// The command completed with an unexpected status code.
    UnexpectedStatus {
        expected: i32,
        actual: i32,
    },
    /// A low-level FFI command returned an error code.
    CommandFailed {
        message: &'static str,
        code: i32,
    },
    UnknownType(&'static str),
    /// Converting Rust strings into C strings failed.
    CString(NulError),
}

impl Display for BulletError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            BulletError::NullPointer(msg) => write!(f, "{msg}"),
            BulletError::ServerUnavailable(msg) => write!(f, "{msg}"),
            BulletError::UnexpectedStatus { expected, actual } => write!(
                f,
                "Unexpected Bullet status. expected={expected} actual={actual}"
            ),
            BulletError::CommandFailed { message, code } => {
                write!(f, "{message} (code={code})")
            }
            BulletError::UnknownType(msg) => write!(f, "{msg}"),
            BulletError::CString(err) => err.fmt(f),
        }
    }
}

impl std::error::Error for BulletError {}

impl From<NulError> for BulletError {
    fn from(value: NulError) -> Self {
        BulletError::CString(value)
    }
}

pub type BulletResult<T> = Result<T, BulletError>;

impl From<io::Error> for BulletError {
    fn from(e: io::Error) -> Self {
        BulletError::CommandFailed {
            message: "IO Error occurred",
            code: e.raw_os_error().unwrap_or(-1),
        }
    }
}

impl From<BulletError> for PhysicsEngineException {
    fn from(e: BulletError) -> Self {
        match e {
            BulletError::NullPointer(msg) => PhysicsEngineException::ServerUnavailable(msg),
            BulletError::ServerUnavailable(msg) => PhysicsEngineException::ServerUnavailable(msg),
            BulletError::UnexpectedStatus { expected, actual } => {
                PhysicsEngineException::CommandFailed(Box::leak(
                    format!("Unexpected status with: expected={expected}, actual={actual}")
                        .into_boxed_str(),
                ))
            }
            BulletError::CommandFailed { message, code } => PhysicsEngineException::CommandFailed(
                Box::leak(format!("{message} (code={code})").into_boxed_str()),
            ),
            BulletError::UnknownType(msg) => PhysicsEngineException::UnknownType(msg),
            BulletError::CString(_) => PhysicsEngineException::NoException,
        }
    }
}

impl From<PhysicsEngineException> for BulletError {
    fn from(e: PhysicsEngineException) -> Self {
        match e {
            PhysicsEngineException::ServerUnavailable(msg) => BulletError::ServerUnavailable(msg),
            PhysicsEngineException::CommandFailed(msg) => BulletError::CommandFailed {
                message: msg,
                code: -1,
            },
            PhysicsEngineException::UnknownType(msg) => BulletError::UnknownType(msg),
            PhysicsEngineException::NoException => BulletError::CommandFailed {
                message: "No exception",
                code: -1,
            },
            PhysicsEngineException::Other(e) => BulletError::CommandFailed {
                message: Box::leak(e.into_boxed_str()),
                code: -1,
            },
        }
    }
}

impl From<BulletError> for RendererException {
    fn from(e: BulletError) -> Self {
        match e {
            BulletError::NullPointer(msg) => RendererException::ServerUnavailable(msg),
            BulletError::ServerUnavailable(msg) => RendererException::ServerUnavailable(msg),
            BulletError::UnexpectedStatus { expected, actual } => {
                RendererException::CommandFailed(Box::leak(
                    format!("Unexpected status with: expected={expected}, actual={actual}")
                        .into_boxed_str(),
                ))
            }
            BulletError::CommandFailed { message, code } => RendererException::CommandFailed(
                Box::leak(format!("{message} (code={code})").into_boxed_str()),
            ),
            BulletError::UnknownType(msg) => RendererException::UnknownType(msg),
            BulletError::CString(_) => RendererException::NoException,
        }
    }
}

impl From<RendererException> for BulletError {
    fn from(e: RendererException) -> Self {
        match e {
            RendererException::ServerUnavailable(msg) => BulletError::ServerUnavailable(msg),
            RendererException::CommandFailed(msg) => BulletError::CommandFailed {
                message: msg,
                code: -1,
            },
            RendererException::UnknownType(msg) => BulletError::UnknownType(msg),
            RendererException::NoException => BulletError::CommandFailed {
                message: "No exception",
                code: -1,
            },
            RendererException::Other(e) => BulletError::CommandFailed {
                message: Box::leak(e.into_boxed_str()),
                code: -1,
            },
        }
    }
}

impl From<BulletError> for RobotException {
    fn from(e: BulletError) -> Self {
        RobotException::CommandException(e.to_string())
    }
}

impl From<RobotException> for BulletError {
    fn from(value: RobotException) -> Self {
        BulletError::CommandFailed {
            message: Box::leak(value.to_string().into_boxed_str()),
            code: -1,
        }
    }
}
