# Third-party crates

Direct dependencies of the engine workspace (`crates/`). `cargo deny check licenses` enforces the
license policy in `deny.toml` for the whole dependency tree; this list is the human summary.

| Crate | Version | License | Used by | For |
|---|---|---|---|---|
| anstream | 1.0 | MIT OR Apache-2.0 | euc-cli | color output that degrades on non-terminals |
| anstyle | 1.0 | MIT OR Apache-2.0 | euc-cli | terminal styles |
| anyhow | 1.0 | MIT OR Apache-2.0 | euc-cli | error reporting in the binary |
| clap | 4.6 | MIT OR Apache-2.0 | euc-cli | command-line parsing |
| crc32fast | 1.5 | MIT OR Apache-2.0 | euc-image | CRC-32/ISO-HDLC of sealed images |
| directories | 6.0 | MIT OR Apache-2.0 | euc-fwsource | per-user cache folder |
| encoding_rs | 0.8 | (Apache-2.0 OR MIT) AND BSD-3-Clause | euc-proto-begode | GBK text the firmware sends |
| hex | 0.4 | MIT OR Apache-2.0 | euc-profile | sha256 text form |
| object | 0.40 | Apache-2.0 OR MIT | euc-image | reading firmware ELF files |
| serde | 1.0 | MIT OR Apache-2.0 | most crates | (de)serialization, `board.toml` instance parameters |
| serde_json | 1.0 | MIT OR Apache-2.0 | euc-fwsource, euc-cli | vendor API replies, `--json` output |
| sha2 | 0.11 | MIT OR Apache-2.0 | euc-profile | sha256 pins |
| thiserror | 2.0 | MIT OR Apache-2.0 | library crates | typed errors |
| toml | 1.1 | MIT OR Apache-2.0 | euc-core, euc-profile, euc-sim, euc-cli | wheel and board profiles |
| ureq | 3.4 | MIT OR Apache-2.0 | euc-fwsource | HTTPS downloads (rustls) |
| tempfile | 3.27 | MIT OR Apache-2.0 | tests only | scratch folders |
| unicorn-engine | =2.1.5 | **GPL-2.0** | euc-unicorn only | CPU backend for development and CI; never in a shipped build (see below) |

Unicorn (QEMU) is GPL-2.0. `deny.toml` allows it for `unicorn-engine` and `unicorn-engine-sys` only and
bans any other crate from depending on them; `euc-unicorn` is outside the workspace default members and
`euc-cli` reaches it only through its `unicorn` feature, which release builds never enable.

Notable licenses deeper in the tree, all allowed by `deny.toml`:

| Crate | License | Pulled in by |
|---|---|---|
| ring | Apache-2.0 AND ISC | ureq → rustls (crypto) |
| rustls-webpki, untrusted | ISC | ureq → rustls (certificate checks) |
| webpki-roots | CDLA-Permissive-2.0 | ureq (Mozilla root certificates) |
| subtle | BSD-3-Clause | sha2 |
| option-ext | MPL-2.0 | directories |
| unicode-ident | (MIT OR Apache-2.0) AND Unicode-3.0 | proc macros |
