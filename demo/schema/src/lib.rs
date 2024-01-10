use eui::*;

#[eui]
#[derive(Debug)]
pub enum Light {
    Off,
    On(Color),
}

#[eui]
#[derive(Debug)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Status for Light {}
impl Command for Light {}
