use eui::*;

#[eui]
#[derive(Debug)]
pub struct Color {
    pins: [bool; 3],
}
pub fn main() {
    let mut b = [0; 200];
    let v = postcard::to_slice(
        &Color {
            pins: [true, true, true],
        },
        &mut b,
    )
    .unwrap();
    dbg!(v.len());
}
