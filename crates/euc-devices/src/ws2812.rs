//! WS2812 LED strip, decoded from the PWM duty values that drive its data line.

/// One LED colour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Rgb {
    /// Red, 0..=255.
    pub r: u8,
    /// Green, 0..=255.
    pub g: u8,
    /// Blue, 0..=255.
    pub b: u8,
}

/// Decodes a WS2812 frame from PWM duty values, one per data bit.
///
/// Firmware usually drives WS2812 LEDs with a timer at 800 kHz whose compare register a DMA
/// channel reloads from a RAM buffer every bit: a long high pulse is a 1, a short one a 0, and a
/// duty of 0 holds the line low (the reset/latch gap). `period` is the timer period in ticks
/// (`ARR + 1`); a bit is 1 when its high time is more than half the period.
///
/// Each LED takes 24 bits, green-red-blue, most significant bit first. Decoding stops at the
/// first zero duty (the latch gap) or the end of the input; a trailing partial LED is dropped.
///
/// The Begode Extreme's strip: TIM3 at 24 MHz with `ARR` 29 (period 30), duties 0x14 for 1 and
/// 0x08 for 0, 32 LEDs, then 1000 zero slots.
///
/// ```
/// use euc_devices::ws2812::{self, Rgb};
///
/// // One LED, green 0x01, red 0x80, blue 0x00.
/// let mut duties = vec![0x08; 24];
/// duties[7] = 0x14; // last green bit
/// duties[8] = 0x14; // first red bit
/// duties.push(0);   // latch
/// assert_eq!(ws2812::decode(duties, 30), [Rgb { r: 0x80, g: 0x01, b: 0x00 }]);
/// ```
pub fn decode(duties: impl IntoIterator<Item = u16>, period: u16) -> Vec<Rgb> {
    let mut leds = Vec::new();
    let mut word = 0_u32;
    let mut bits = 0;
    for duty in duties {
        if duty == 0 {
            break;
        }
        word = word << 1 | u32::from(u32::from(duty) * 2 > u32::from(period));
        bits += 1;
        if bits == 24 {
            let [_, g, r, b] = word.to_be_bytes();
            leds.push(Rgb { r, g, b });
            (word, bits) = (0, 0);
        }
    }
    leds
}

#[cfg(test)]
mod tests {
    use super::*;

    fn duties_for(colours: &[Rgb]) -> Vec<u16> {
        colours
            .iter()
            .flat_map(|c| {
                let word = u32::from_be_bytes([0, c.g, c.r, c.b]);
                (0..24).rev().map(move |bit| if word >> bit & 1 == 1 { 0x14 } else { 0x08 })
            })
            .collect()
    }

    #[test]
    fn decodes_grb_msb_first() {
        let strip = [Rgb { r: 0x10, g: 0x00, b: 0x00 }, Rgb { r: 0xFF, g: 0x80, b: 0x01 }, Rgb::default()];
        let mut duties = duties_for(&strip);
        duties.extend([0; 1000]);
        assert_eq!(decode(duties, 30), strip);
    }

    #[test]
    fn stops_at_the_latch_and_drops_partial_leds() {
        let mut duties = duties_for(&[Rgb { r: 1, g: 2, b: 3 }]);
        duties.extend([0x14; 10]);
        assert_eq!(decode(duties.clone(), 30).len(), 1);
        duties.insert(24, 0);
        duties.extend(duties_for(&[Rgb { r: 9, g: 9, b: 9 }]));
        assert_eq!(decode(duties, 30), [Rgb { r: 1, g: 2, b: 3 }]);
    }

    #[test]
    fn threshold_is_half_the_period() {
        assert_eq!(decode([15; 24], 30), [Rgb::default()], "exactly half is a 0");
        assert_eq!(decode([16; 24], 30), [Rgb { r: 0xFF, g: 0xFF, b: 0xFF }]);
    }
}
