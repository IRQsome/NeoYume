<h1><img src="stuff/logobig.png" alt="NEO⋅YUME"></img>
<p align="center">
MAX 256 MEGA<br>
PROP-GEAR SPEC
</p>
</h1>

## About

NeoYume is a NeoGeo AES emulator for Parallax P2 micro-controllers with appropriate external memory
(Currently supported: 4xQPI PSRAM, as on [P2-EC32MB](https://www.parallax.com/product/p2-edge-module-with-32mb-ram/) (owie ouch, that retail price), 2xQPI PSRAM, 1xQPI PSRAM (not recommended), 1xHyperRAM).

Features include:
  - Low latency framebuffer-less high-level emulation
  - Pretty good audio
  - Video output over VGA, HDMI, YPbPr, S-video, Composite or 6bit RGB bus (ILI9342 LCD)
  - USB input (Keyboard and select gamepads, see below) or direct hookup to discrete switches.
  - Bugs. But not _that_ many.

Forum thread, chronicalling development and other interesting info: https://forums.parallax.com/discussion/comment/1538689/#Comment_1538689

## Usage

### SD Setup

Note: all filenames and paths are case-insensitive. The FAT driver without LFN always sees uppercase 8.3 names.

- Make sure your card is formatted to FAT32
- Create a `NEOYUME` directory on the root of the card
- Place your `neo-epo.bin` (or `neo-po.bin` for the Japanese BIOS) into `/NEOYUME/NEOGEO/` (i.e. `/NEOYUME/NEOGEO/NEO-EPO.BIN`)
- Unzip your MAME-style game ROMs into the `NEOYUME` directory, such that the paths look like this: `/NEOYUME/MSLUG/201-C1.BIN`. **All files must have a .bin extension, rename them if they have .c1,.v1,etc kinda extensions!**
- Some games have multiple romset variations. You will need the one matching the game list entry.
- Some games (noted in the compatibility list) require decryption/patching. To do this, use the romfix tool (see below)

#### Using the romfix tool
`romfix.rb` is a Ruby script that automatically decrypts/patches ROMs that need it. As a bonus, it also takes care of any renaming that may be neccessary.
To use it:
 - Make sure you have a Ruby interpreter installed (On Windows, you can use [RubyInstaller](https://rubyinstaller.org))
 - Copy `romfix.rb` into your NEOYUME directory.
 - Open a command prompt there and run `ruby ./romfix.rb`. You may also use `ruby ./romfix.rb --cleanup` to have it automatically delete unneeded files.

#### Notes on acquiring ROMs legally

As an alternative to illicit acquisition of game ROMs, you may also:
 - Purchase any SNK NeoGeo game rerelease on gog.com
 - Download the MacOS X version, which is a ZIP file
 - Unzip it
 - Find the ROMset inside (for Metal Slug, it is at `MetalSlug.app/Contents/Resources/mslug.zip`)
 - Proceed as usual (don't foget to change file extensions to .bin).

Note that the AES BIOS ROM (neo-epo.bin) is not included, so you'll have to find it elsewhere (don't feel bad, for they did for some reason (read: they probably just got a BIOS romset from a shady site themselves) include unibios, which is against its terms of use (then again, unibios itself exists in violation of copyright)).

tl;dr; No sane soul gives a singular shart about BIOS copyrights, just download it. Be gay do crime.

Also note that I have not tested every single available game, YMMV

### Compiling

- Configure video, audio, input and RAM in `config.spin2`
  - The comments hopefully explain the settings
  - See [RAMCONFIG.MD](RAMCONFIG.MD) for details on the RAM configuration
- Make sure you have [flexspin](https://github.com/totalspectrum/spin2cpp) 5.9.27 or higher installed
- Run `build.sh`
- Load the resulting `neoyume_lower.binary`

## Game Compatibility

Due to the nature of NeoGeo ROM dumps, the emulator requires prior knowledge of all games. Thus games not listed here (or rather, `neoyume_gamedb.spin2`) just will not even try to load.

Note 1: I haven't tested every game thoroughly. Don't complain if the victory screen crashes and burns your house down.

Note 2: I sometimes gloss over very minor issues that don't significantly impact the enjoyment and/or are somewhat difficult to reproduce.

|ROMset name|Title|RAM Required|Compatibility|
|-|-|-:|-|
|`2020bb`|2020 Super Baseball|8 MB|Works|
|`3countb`|3 Count Bout|16 MB|Works|
|`aodk`|Aggressors of Dark Kombat|24 MB|Works|
|`alpham2`|Alpha Mission 2|8 MB|Works|
|`androdun`|Andro Dunos|8 MB|Works|
|`aof`|Art of Fighting|16 MB|Works|
|`aof2`|Art of Fighting 2|24 MB|Works|
|`aof3`|Art of Fighting 3|**48 MB**|Works|
|`bakatono`|Bakatonosama Mahjong Manyuuki|8 MB|Works|
|`b2b`|Bang Bang Busters|8 MB|Works|
|`bangbead`|Bang Bead|24 MB|Works (Needs decrypted C)|
|`bstars`|Baseball Stars|8 MB|Works|
|`bstars2`|Baseball Stars 2|8 MB|Works|
|`flipshot`|Battle Flip Shot|8 MB|Works|
|`blazstar`|Blazing Star|**48 MB**|Works|
|`bjourney`|Blue's Journey|8 MB|Works|
|`breakers`|Breakers|32 MB|Works|
|`breakrev`|Breakers Revenge|**48 MB**|Works|
|`burningf`|Burning Fight|8 MB|Works|
|`ctomaday`|Captain Tomaday|16 MB|Works|
|`crswords`|Crossed Swords|8 MB|Works|
|`crswd2bl`|Crossed Swords 2 (Unofficial conversion)|16 MB|Works|
|`cyberlip`|Cyber-Lip|8 MB|Works|
|`doubledr`|Double Dragon|24 MB|Works|
|`eightman`|Eight Man|8 MB|Works|
|`fatfury1`|Fatal Fury|8 MB|Works|
|`fatfury2`|Fatal Fury 2|16 MB|**Unemulated copy protection (#11)**|
|`fatfursp`|Fatal Fury Special|24 MB|Works|
|`fatfury3`|Fatal Fury 3|**48 MB**|Works|
|`fightfev`|Fight Fever|16 MB|Works|
|`fbfrenzy`|Football Frenzy|8 MB|Works|
|`galxyfg`|Galxy Fight - Universal Warriors|16 MB|**Graphical Issues**|
|`ganryu`|Ganryu|24 MB|Works (Needs decrypted C)|
|`ghostlop`|Ghostlop|16 MB|Works|
|`gpilots`|Ghost Pilots|8 MB|Works|
|`goalx3`|Goal! Goal! Goal!|16 MB|Works|
|`gururin`|Gururin|8 MB|Works|
|`ironclad`|Ironclad|24 MB|Works (need to rename files to fit in 8.3 (remove "proto_" prefix))|
|`janshin`|Janshin Densetsu - Quest of the Jongmaster|8 MB|Works|
|`kabukikl`|Far East of Eden - Kabuki Klash|32 MB|Works|
|`karnovr`|Karnov's Revenge|16 MB|Works|
|`kof94`|King of Fighters '94|32 MB|Works|
|`kof95`|King of Fighters '95|**48 MB**|Works|
|`kof96`|King of Fighters '96|**48 MB**|Works|
|`kof97`|King of Fighters '97|**64 MB**|Works|
|`kof98h`|King of Fighters '98|**96 MB**|Works|
|`kof99ka`|King of Fighters '99|**96 MB**|Works (Needs decrypted C)|
|`kof2000n`|King of Fighters 2000|**96 MB**|**Broken fix layer** (Needs decrypted C/M)|
|`kof2001h`|King of Fighters 2001|**96 MB**|Works (Needs decrypted C/M)|
|`kof2002`|King of Fighters 2002|**96 MB**|Works (Needs decrypted C/V/M)|
|`kotmh`|King of the Monsters|8 MB|Works|
|`kotm2`|King of the Monsters 2|16 MB|Works|
|`kizuna`|Kizuna Encounter|**48 MB**|Works|
|`lastblad`|The Last Blade|**64 MB**|Works|
|`lastbld2`|The Last Blade 2|**80 MB**|Works|
|`lresort`|Last Resort|8 MB|Works|
|`lbowling`|League Bowling|4 MB|Works|
|`legendos`|Legend of Success Joe|8 MB|Works|
|`mahretsu`|Mahjong Kyo Retsuden|8 MB|Works|
|`magdrop2`|Magical Drop 2|16 MB|Works|
|`magdrop3`|Magical Drop 3|24 MB|Works|
|`maglord`|Magician Lord|8 MB|Works|
|`marukodq`|Chibi Maruko-chan: Maruko Deluxe Quiz|16 MB|Works|
|`mosyougi`|Master of Shōgi|8 MB|Works (minor gfx oddity)|
|`matrim`|Matrimelee|**96 MB**|Works (Needs decrypted C/V/M)|
|`mslug`|Metal Slug|32 MB|Works|
|`mslug2`|Metal Slug 2|**48 MB**|Works|
|`mslugx`|Metal Slug X|**64 MB**|Works (Needs patched P)|
|`mslug3h`|Metal Slug 3|**96 MB**|**Sound issues** (512k M1, #8) (Needs decrypted C)|
|`mslug4h`|Metal Slug 4|**80 MB**|Works (Needs decrypted C/M)|
|`mslug5h`|Metal Slug 5|**96 MB**|Works (Needs decrypted C/V/P/M)|
|`minasan`|Minasan no Okagesamadesu!|8 MB|Works|
|`miexchng`|Money Puzzle Exchanger|16 MB|Works|
|`mutnat`|Mutation Nation|8 MB|Works|
|`nam1975`|NAM-1975|8 MB|Works|
|`neobombe`|Neo Bomberman|24 MB|Works|
|`neodrift`|Neo Drift Out|16 MB|**Graphical issues** (#4)|
|`neocup98`|Neo-Geo Cup '98 - The Road to the Victory|32 MB|**Graphical issues** (#4 (???))|
|`neomrdo`|Neo Mr. Do!|8 MB|Works|
|`turfmast`|Neo Turf Masters|24 MB|**Graphical issues** (Needs Timer IRQ)|
|`nitd`|Nightmare in the Dark|24 MB|Works (Needs decrypted C)|
|`ncombat`|Ninja Combat|8 MB|Works|
|`ncommand`|Ninja Commando|8 MB|Works|
|`ninjamas`|Ninja Master's|**48 MB**|Works|
|`overtop`|Over Top|32 MB|Works|
|`panicbom`|Panic Bomber|8 MB|Works|
|`pgoal`|Pleasure Goal|24 MB|Works|
|`pnyaa`|Pochi and Nyaa|24 MB|Works (Needs decrypted C/M)|
|`popbounc`|Pop n' Bounce|8 MB|Works|
|`pspikes2`|Power Spikes 2|16 MB|Works|
|`preisle2`|Prehistoric Isle 2|**64 MB**|Works (Needs decrypted C)|
|`pulstar`|Pulstar|**48 MB**|Works|
|`pbobbl2n`|Puzzle Bobble 2|24 MB|Works|
|`puzzledp`|Puzzle de Pon|4 MB|Works|
|`puzzldpr`|Puzzle de Pon R|4 MB|Works|
|`quizdais`|Quiz Daisousa Sen - The Last Count Down|8 MB|Works|
|`quizdai2`|Quiz Meitantei Neo & Geo - Quiz Daisousa Sen Part 2|8 MB|Works|
|`quizkof`|Quiz King of Fighters|16 MB|Works|
|`joyjoy`|Puzzled|4 MB|Works|
|`rotdh`|Rage of the Dragons|**96 MB**|Works (Needs decrypted C/M)|
|`rbff1`|Real Bout Fatal Fury|**48 MB**|Works|
|`rbffspec`|Real Bout Fatal Fury Special|**64 MB**|Works|
|`rbff2`|Real Bout Fatal Fury 2|**80 MB**|Works|
|`ragnagrd`|Ragnagard|**48 MB**|Works|
|`ridhero`|Riding Hero|8 MB|**Graphical issues**|
|`roboarmy`|Robo Army|8 MB|Works|
|`samsho`|Samurai Shodown|24 MB|Works|
|`samsho2`|Samurai Shodown 2|32 MB|Works|
|`samsho3h`|Samurai Shodown 3|**48 MB**|Works|
|`samsho4`|Samurai Shodown 4|**48 MB**|Works|
|`samsho5h`|Samurai Shodown 5|**96 MB**|Works (Needs decrypted C/V/M)|
|`samsh5sp`|Samurai Shodown 5 Special|**96 MB**|Works (Needs decrypted C/V/M)|
|`savagere`|Savage Reign|32 MB|Works|
|`sengoku`|Sengoku|8 MB|Works|
|`sengoku2`|Sengoku 2|16 MB|Works|
|`sengoku3`|Sengoku 3|**80 MB**|Works (Needs decrypted C)|
|`shocktro`|Shock Troopers|**48 MB**|Works|
|`shocktr2`|Shock Troopers 2nd Squad|**64 MB**|Works|
|`svc`|SNK vs. Capcom - SVC Chaos|**96 MB**|**Black screen**|
|`socbrawl`|Soccer Brawl|8 MB|Works|
|`sonicwi2`|Sonic Wings 2|16 MB|Works|
|`sonicwi3`|Sonic Wings 3|24 MB|Works|
|`spinmast`|Spinmaster|16 MB|**Black screen** (Timer IRQ, #5)|
|`stakwin`|Stakes Winner|16 MB|Works|
|`stakwin2`|Stakes Winner 2|24 MB|Works|
|`strhoop`|Street Hoop|16 MB|Works|
|`s1945p`|Strikers 1945 Plus|**96 MB**|Works (Needs decrypted C)|
|`sdodgeb`|Super Dodge Ball|24 MB|Works|
|`ssideki`|Super Sidekicks|16 MB|**Unemulated copy protection (#11)**|
|`ssideki2`|Super Sidekicks 2|16 MB|**Graphical issues**|
|`ssideki2`|Super Sidekicks 3|24 MB|**Graphical issues**|
|`superspy`|The Super Spy|8 MB|Works|
|`tws96`|Tecmo World Soccer '96|16 MB|Works|
|`tophuntr`|Top Hunter - Roddy and Cathy|16 MB|Works|
|`tpgolf`|Top Player's Golf|8 MB|**Graphical Issues** (Also may need romfix)|
|`trally`|Thrash Rally|8 MB|**Graphical issues**|
|`twinspri`|Twinkle Star Sprites|24 MB|Works|
|`ssideki4`|The Ultimate 11|32 MB|**Graphical issues**|
|`viewpoint`|Viewpoint|16 MB|Works|
|`gowcaizr`|Voltage Fighter - Gowcaizer|24 MB|Works|
|`wakuwak7`|Waku Waku 7|**48 MB**|Works|
|`wh1h`|World Heroes|16 MB|Works|
|`wh2`|World Heroes 2|24 MB|Works|
|`wh2j`|World Heroes 2 Jet|24 MB|Works|
|`whp`|World Heroes Perfect|**48 MB**|Works|
|`wjammers`|Windjammers|16 MB|Works|
|`zedblade`|Zed Blade|16 MB|Works|
|`zupapa`|Zupapa!|24 MB|Works (Needs decrypted C)|

Note: RAM requirements are rounded to the next highest of 4MB, 8MB, 16MB, 24MB, 32MB, 48MB, 64MB, 80MB, 96MB

## USB Controller compatibility

Currently four types of USB controller are supported, see below sections.

Currently only one USB port is supported, so no 2 player mode (yes, no 2 player in a neo geo emu. complain that the USB driver is less than ideal.)

### Keyboard

Boot protocol / 6KRO mode only!

Keymap (assuming QWERTY):
|Keyboard|NeoGeo|
|:-:|:-:|
|<kbd>Arrow Keys</kbd>|Joystick|
|<kbd>Z</kbd>|A|
|<kbd>X</kbd>|B|
|<kbd>C</kbd>|C|
|<kbd>V</kbd>|D|
|<kbd>Enter</kbd>|Start|
|<kbd>Backspace</kbd>|Select|

### Generic XInput gamepad

Not sure to what extent it works with actual XBox controllers, but it sure works with cheapo pads and the semi-secret XInput mode on an 8bitdo Retro Reciever.

Keymap emulates layout of a NeoGeo mini controller, thus:
|XInput|NeoGeo|
|:-:|:-:|
|D-Pad or Left Analog|Joystick|
|X|A|
|A|B|
|Y|C|
|B|D|
|Start|Start|
|Back|Select|

### NeoGeo mini controller

Detected by PID 0x1112 VID 0x0E6F.

Keymap should be obvious.

### RetroBit SEGA Saturn/Megadrive controller

Detected by PID 0x0011 VID 0x0079.

Keymap:
|SEGA|NeoGeo|
|:-:|:-:|
|D-Pad|Joystick|
|A|A|
|B|B|
|C or X|C|
|Y or Z|D|
|Start|Start|
|Mode (MD only)|Select|

## About

NeoYume was programmed almost entirely by me, Ada Gottensträter, though taking clues from existing projects (mostly MAME).
The AY/SSG core is adapted from work by Johannes Ahlebrand.
The USB driver was written by Garry Jordan, with additional work by Marco Maccaferri,
The loader memory driver (`psram16drv.spin2` and friends) was written by Roger Loh.

