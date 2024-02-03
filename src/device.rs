// TODO: Is there a nicer way for this default device impl without a macro? Need generics on modules
// TODO: don't actually export this?
#[macro_export]
macro_rules! default_device_impl {
    ($struct: ident, $command: ident, $status: ident) => {
        impl crate::Device for $struct {
            type Command = $command;
            type Status = $status;

            fn pdi_window_size(&self) -> usize {
                core::cmp::max(
                    Option::<Self::Command>::POSTCARD_MAX_SIZE,
                    Self::Status::POSTCARD_MAX_SIZE,
                )
            }

            fn status(&self, pdi_window: &[u8]) -> Self::Status {
                postcard::from_bytes(&pdi_window).unwrap()
            }

            fn command(&self, pdi_window: &mut [u8], cmd: &Self::Command) {
                postcard::to_slice(&Some(cmd), pdi_window).unwrap();
            }
        }
    };
}
pub mod led {
    pub use postcard;
    use postcard::experimental::max_size::MaxSize;
    use serde::{Deserialize, Serialize};

    #[cfg(feature = "eui")]
    use eui::*;

    #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
    #[cfg_attr(feature = "eui", derive(eui::eui_derive::Schema))]
    pub enum Light {
        Off,
        On(Color),
    }

    #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
    #[cfg_attr(feature = "eui", derive(eui::eui_derive::Schema))]
    pub struct Color {
        pub r: u8,
        pub g: u8,
        pub b: u8,
    }

    pub struct Led {}

    pub const PDI_WINDOW_SIZE: usize = Option::<Light>::POSTCARD_MAX_SIZE;

    impl crate::Device for Led {
        type Command = Light;
        type Status = Light;

        fn pdi_window_size(&self) -> usize {
            PDI_WINDOW_SIZE
        }

        fn status(&self, pdi_window: &[u8]) -> Self::Status {
            postcard::from_bytes(&pdi_window).unwrap()
        }

        fn command(&self, pdi_window: &mut [u8], cmd: &Self::Command) {
            postcard::to_slice(&Some(cmd), pdi_window).unwrap();
        }
    }
}
