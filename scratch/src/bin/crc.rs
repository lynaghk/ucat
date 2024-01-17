pub const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);

pub fn main() {
    dbg!(CRC.checksum(b"12345678"));

    let mut d = CRC.digest();
    for bs in [b"1234", b"5678"] {
        d.update(bs);
    }
    dbg!(d.finalize());
}
