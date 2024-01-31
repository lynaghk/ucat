// TODO: Is there a nicer way for this default device impl without a macro? Need generics on modules
// TODO: don't actually export this?
#[macro_export]
macro_rules! device_impl {
    () => {
        pub struct Device {
            pub pdi_offset: usize,
        }

        impl crate::Device for Device {
            type Status = Status;
            type Command = Command;

            fn status(&self, pdi: &[u8]) -> Status {
                postcard::from_bytes(&pdi[self.pdi_offset..(self.pdi_offset + PDI_WINDOW_SIZE)])
                    .unwrap()
            }

            fn command(&self, pdi: &mut [u8], cmd: Option<&Command>) {
                postcard::to_slice(
                    &cmd,
                    &mut pdi[self.pdi_offset..(self.pdi_offset + PDI_WINDOW_SIZE)],
                )
                .unwrap();
            }
        }
    };
}

pub mod led {
    pub use postcard;
    use postcard::experimental::max_size::MaxSize;
    use serde::{Deserialize, Serialize};
    pub const PDI_WINDOW_SIZE: usize = Option::<Light>::POSTCARD_MAX_SIZE;

    #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
    pub enum Light {
        Off,
        On(Color),
    }

    #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
    pub struct Color {
        pub r: u8,
        pub g: u8,
        pub b: u8,
    }

    pub type Command = Light;
    pub type Status = Light;

    crate::device_impl!();
}
