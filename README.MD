<h1 style="display: none;">NeoYume Emulator</h1>

```
  ###    ### #########   ######       #####    ### ####   ### ###          ### ######### 
   ###    #   ##     #  ##    ##        ##     #    ##     #   ###        ###   ##     # 
   # ##   #   ##  #    ##      ##    #   ##   #     ##     #   # ##      # ##   ##  #    
   #  ##  #   #####   ###      ###  ###   ## #      ##     #   #  ##    #  ##   #####    
   #   ## #   ##  #    ##      ##    #     ##       ##     #   #   ##  #   ##   ##  #    
   #    ###   ##      # ##    ##           ##        ##   #    #    ###    ##   ##      #
  ###    ### #########   ######           ####        ####    ###    #    #### ######### 

                              ＭＡＸ　２５６　ＭＥＧＡ
                            ＰＲＯＰ－ＧＥＡＲ　ＳＰＥＣ
```

NeoYume is a NeoGeo AES emulator for Parallax P2 micro-controllers with appropriate external memory
(Currently supported: 4xQPI PSRAM, as on [P2-EC32MB](https://www.parallax.com/product/p2-edge-module-with-32mb-ram/) (owie ouch, that retail price)).

Features include:
  - Low latency framebuffer-less high-level emulation
  - Pretty good audio
  - Video output over VGA, HDMI, YPbPr, S-video or Composite
  - USB Keyboard input (also select gamepads, see below)
  - Bugs.
  - PLAYS METÄL SLÜG!!!einself!"!!!

## Usage

### SD Setup

Note: all filenames and paths are case-insensitive. The FAT driver without LFN always sees uppercase 8.3 names.

- Create a `NEOYUME` directory on the root of the card
- Place your `neo-epo.bin` (or `neo-po.bin` if `USE_JP_BIOS` is set in config) into `/NEOYUME/NEOGEO/` (i.e. `/NEOYUME/NEOGEO/NEO-EPO.BIN`)
- Unzip your MAME-style game ROMs into the `NEOYUME` directory, such that the paths look like this: `/NEOYUME/MSLUG/201-C1.BIN`. **All files must have a .bin extension, rename them if they have .c1,.v1,etc kinda extensions!**
- Some games (noted in the compatibility list) need pre-decrypted ROMs. (Decrypting the normal MAME sets on load is possible, but would be extremely slow)

#### Notes on acquiring ROMs legally

As an alternative to illicit acquisition of game ROMs, you may also:
 - Purchase any SNK NeoGeo game rerelease on gog.com
 - Download the MacOS X version, which is a ZIP file
 - Unzip it
 - Find the ROMset inside (for Metal Slug, it is at `MetalSlug.app/Contents/Resources/mslug.zip`)
 - Proceed as usual (don't foget to change file extensions to .bin).

Note that the AES BIOS ROM (neo-epo.bin) is not included, so you'll have to find it elsewhere (don't feel bad, for they did for some reason (read: they probably just got a BIOS romset from a shady site themselves) include unibios, which is against its terms of use (then again, unibios itself exists in violation of copyright)).

Also note that I have not tested every single available game, YMMV

tl;dr; No sane soul gives a singular shart about BIOS copyrights, just download it. Be gay do crime.

### Compiling

- Configure video, audio and USB pins in `config.spin2`
- Make sure you have flexspin 5.9.12 or higher installed
- Run `build.sh`
- Load the resulting `neoyume_lower.binary`

## Game Compatibility

These are just the games I tested, to the extent that I tested them. Don't complain if the victory screen crashes and burns your house down.

Note 1: I sometimes gloss over very minor issues that don't significantly impact the enjoyment. ~~In particular, some games rarely have a "wrong sound" issue wherein an ADPCM channel runs through all consecutive data in ROM until another actual sound is triggered - except for samsho3/samsho4, this happens rarely enough that it is very difficult to even verify.~~ Issue is fixed now. I hope.

Note 2: There currently isn't a public tool for V ROM decryption, thus all games with encrypted V are marked as GAME_ISSUE in the internal database, even if they actually work fine.

|ROMset name|Title|RAM Required|Compatibility|
|-|-|-:|-|
|`aodk`|Aggressors of Dark Kombat|24 MB|**Works**|
|`aof`|Art of Fighting|16 MB|**Works**|
|`blazstar`|Blazing Star|48 MB|Broken sound|
|`ctomaday`|Captain Tomaday|16 MB|**Works**|
|`crswords`|Crossed Swords|8 MB|**Works**|
|`cyberlip`|Cyber-Lip|8 MB|**Works**|
|`doubledr`|Double Dragon|24 MB|**Works**|
|`fatfury1`|Fatal Fury|8 MB|**Works**|
|`fatfursp`|Fatal Fury Special|24 MB|**Works**|
|`ganryu`|Ganryu|24 MB|**Works** (Needs decrypted C)|
|`ironclad`|Ironclad|24 MB|**Works** (need to rename files to fit in 8.3 (remove "proto_" prefix))|
|`kabukikl`|Far East of Eden - Kabuki Klash|32 MB|**Works**|
|`karnovr`|Karnov's Revenge|16 MB|**Works**|
|`kof94`|King of Fighters '94|32 MB|**Works**|
|`kof95`|King of Fighters '95|**48 MB**|**Works**|
|`kof96`|King of Fighters '96|**48 MB**|**Works**|
|`kof97`|King of Fighters '97|**64 MB**|**Works**|
|`kof98`|King of Fighters '98|**96 MB**|**Works** (Needs decrypted C)|
|`kof99`|King of Fighters '99|**96 MB**|**Works** (Needs decrypted C)|
|`kof2000`|King of Fighters 2000|**96 MB**|Broken fix layer (Needs decrypted C)|
|`kof2001`|King of Fighters 2001|**96 MB**|**Works** (Needs decrypted C)|
|`kof2002`|King of Fighters 2002|**96 MB**|Works  (Needs decrypted C and V)|
|`kotmh`|King of the Monsters|8 MB|**Works**|
|`kizuna`|Kizuna Encounter|**48 MB**|**Works**|
|`lastblad`|The Last Blade|**64 MB**|**Works**|
|`lastbld2`|The Last Blade 2|**80 MB**|**Works**|
|`magdrop2`|Magical Drop 2|16 MB|**Works**|
|`magdrop3`|Magical Drop 3|24 MB|**Works**|
|`matrim`|Matrimelee|**96 MB**|**Works** (Needs decrypted C/V)|
|`mslug`|Metal Slug|32 MB|**Works**|
|`mslug2`|Metal Slug 2|**48 MB**|**Works**|
|`mslugx`|Metal Slug X|**64 MB**|**Works**|
|`mslug3`|Metal Slug 3|**96 MB**|Sound issues (512k M1) (Needs decrypted C)|
|`mslug4`|Metal Slug 4|**96 MB**|**Works** (Needs decrypted C)|
|`mutnat`|Mutation Nation|8 MB|**Works**|
|`neobombe`|Neo Bomberman|24 MB|**Works**|
|`neodrift`|Neo Drift Out|16 MB|Glitching graphics, sound seems to die after one game|
|`pulstar`|Pulstar|48 MB|Broken Sound|
|`pbobbl2n`|Puzzle Bobble 2|24 MB|**Works**|
|`samsho`|Samurai Shodown|24 MB|**Works**|
|`samsho2`|Samurai Shodown 2|32 MB|**Works**|
|`samsho3`|Samurai Shodown 3|**48 MB**|**Works**|
|`samsho4`|Samurai Shodown 4|**48 MB**|**Works**|
|`samsho5`|Samurai Shodown 5|**96 MB**|**Works** (Needs decrypted C and V)|
|`samsh5sp`|Samurai Shodown 5 Special|**96 MB**|**Works** (Needs decrypted C and V)|
|`sengoku`|Sengoku|8 MB|**Works**|
|`sengoku2`|Sengoku 2|16 MB|**Works**|
|`sengoku3`|Sengoku 3|**64 MB**|**Works**|
|`sengoku3`|Sengoku 3|**64 MB**|**Works** (Needs decrypted C)|
|`shocktro`|Shock Troopers|**48 MB**|**Works**|
|`shocktr2`|Shock Troopers 2nd Squad|**64 MB**|**Works**|
|`sonicwi2`|Sonic Wings 2|16 MB|**Works**|
|`sonicwi3`|Sonic Wings 3|24 MB|**Works**|
|`strhoop`|Street Hoop|16 MB|**Works**|
|`spinmast`|Spinmaster|16 MB|Black screen after eyecatcher|
|`twinspri`|Twinkle Star Sprites|24 MB|**Works**|
|`viewpoint`|Viewpoint|16 MB|**Works**|
|`wakuwak7`|Waku Waku 7|**48 MB**|**Works**|
|`wh2`|World Heroes 2|24 MB|**Works**|
|`wjammers`|Windjammers|16 MB|**Works**|
|`zedblade`|Zed Blade|16 MB|**Works**|

Note: RAM requirements are rounded to the next highest of 8MB, 16MB, 24MB, 32MB, 48MB, 64MB, 80MB, 96MB

## USB Controller compatibility

Currently four types of USB controller are supported, see below sections.

Currently only one USB port is supported, so no 2 player mode (yes, no 2 player in a neo geo emu. complain that the USB driver sucks.)

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

NeoYume was programmed almost entirely by me, Ada Gottensträter.
The AY/SSG core is adapted from work by Johannes Ahlebrand.
The USB driver was written by Garry Jordan, with additional work by Marco Maccaferri,
The loader memory driver (`psram16drv.spin2` and friends) was written by Roger Loh.

