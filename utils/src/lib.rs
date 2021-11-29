#![cfg_attr(not(test), no_std)]

pub mod time;

pub fn num_length(n: usize) -> usize {
    if n > 9 {
        num_length(n / 10) + 1
    } else {
        1
    }
}

pub fn num_to_string(num: usize) -> (usize, [u8; 10]) {
    let lng = num_length(num);
    let mut value = num;

    let mut buf = [0; 10];
    for i in 0..lng {
        let dig = value % 10;
        value /= 10;

        let idx = (lng - i) as usize - 1;
        buf[idx] = b'0' + dig as u8;
    }
    (lng, buf)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn num_length_test() {
        assert_eq!(num_length(0), 1);
        assert_eq!(num_length(1), 1);
        assert_eq!(num_length(9), 1);
        assert_eq!(num_length(10), 2);
        assert_eq!(num_length(99), 2);
        assert_eq!(num_length(100), 3);
        assert_eq!(num_length(999), 3);
        assert_eq!(num_length(1000), 4);
        assert_eq!(num_length(10000), 5);
        assert_eq!(num_length(100000), 6);
        assert_eq!(num_length(1000000), 7);
        assert_eq!(num_length(10000000), 8);
        assert_eq!(num_length(999999999), 9);
        assert_eq!(num_length(0xffffffff), 10);
    }

    #[test]
    fn num_to_string_test() {
        let (lng, out) = num_to_string(123);
        assert_eq!(lng, 3);
        assert_eq!(out[0], '1' as u8);
        assert_eq!(out[1], '2' as u8);
        assert_eq!(out[2], '3' as u8);
        assert_eq!(out[3], 0);
    }
}
