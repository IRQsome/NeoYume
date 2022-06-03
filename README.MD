<h1 style="display: none;">NeoYume Emulator</h1>
<pre>
  ###    ### #########   ######       #####    ### ####   ### ###          ### ######### 
   ###    #   ##     #  ##    ##        ##     #    ##     #   ###        ###   ##     # 
   # ##   #   ##  #    ##      ##    #   ##   #     ##     #   # ##      # ##   ##  #    
   #  ##  #   #####   ###      ###  ###   ## #      ##     #   #  ##    #  ##   #####    
   #   ## #   ##  #    ##      ##    #     ##       ##     #   #   ##  #   ##   ##  #    
   #    ###   ##      # ##    ##           ##        ##   #    #    ###    ##   ##      #
  ###    ### #########   ######           ####        ####    ###    #    #### ######### 

                              ＭＡＸ　２５６　ＭＥＧＡ
                            ＰＲＯＰ－ＧＥＡＲ　ＳＰＥＣ
</pre>

NeoYume is a NeoGeo AES emulator for Parallax P2 micro-controllers with appropriate external memory
(Currently supported: 4xQPI PSRAM, as on [P2-EC32MB](https://www.parallax.com/product/p2-edge-module-with-32mb-ram/) (owie ouch, that retail price)).

Features include:
  - Low latency framebuffer-less high-level emulation
  - Pretty good audio
  - Video output over VGA, HDMI, S-video or Composite
  - USB Keyboard input (also select gamepads, see below)
  - Bugs.
  - PLAYS METÄL SLÜG!!!einself!"!!!

## Usage

### SD Setup

Note: all filenames and paths are case-insensitive. The FAT driver without LFN always sees uppercase 8.3 names.

- Create a `NEOYUME` directory on the root of the card
- Place your `neo-epo.bin` (or `neo-po.bin` if `USE_JP_BIOS` is set in config) into `/NEOYUME/NEOGEO/` (i.e. `/NEOYUME/NEOGEO/NEO-EPO.BIN`)
- Unzip your MAME-style game ROMs into the `NEOYUME` directory, such that the paths look like this: `/NEOYUME/MSLUG/201-C1.BIN`.

### Compiling

- Configure video, audio and USB pins in `config.spin2`
- Make sure you have flexspin 5.9.12 or higher installed (not released at the time of writing, use git build for now)
- Run `build.sh`
- Load the resulting `neoyume_lower.binary`

## Game Compatibility

Note that due to RAM size, late-era games over 255 megabit _can't work_, though they can be tested with missing graphics (see mslug2).

These are just the games I tested, to the extent that I tested them. Don't complain if the victory screen crashes and burns your house down.

|ROMset name|Title|Compatibility|
|-|-|-|
|`crswords`|Crossed Swords|**Works**|
|`cyberlip`|Cyber-Lip|Doesn't boot|
|`doubledr`|Double Dragon|**Works**|
|`fatfury`|Fatal Fury|**Works**|
|`ironclad`|Ironclad|**Works** (need to rename files to fit in 8.3 (remove "proto_" prefix))|
|`karnovr`|Karnov's Revenge|Black screen after eyecatcher|
|`kof95`|King of Fighters '95|Works in theory, in practice the memory map is too holey to fit|
|`kotmh`|King of the Monsters|**Works**|
|`magdrop2`|Magical Drop 2|Broken sound|
|`mslug`|Metal Slug|**Works**|
|`mslug2`|Metal Slug 2|**Would work** (needs bigger PSRAM to load all graphics)|
|`mutnat`|Mutation Nation|**Works**|
|`neodrift`|Neo Drift Out|Glitching graphics, sound seems to die after one game|
|`pbobbl2n`|Puzzle Bobble 2|**Works**|
|`samsho`|Samurai Shodown|**Works**|
|`samsho2`|Samurai Shodown 2|**Works**|
|`sengoku`|Sengoku|**Works**|
|`sengoku2`|Sengoku 2|Hangs quickly|
|`sonicwi2`|Sonic Wings 2|Broken sound|
|`sonicwi3`|Sonic Wings 3|Broken sound|
|`twinspri`|Twinkle Star Sprites|**Works**|
|`viewpoint`|Viewpoint|Hangs immediately|
|`wakuwak7`|Waku Waku 7|Would be too big, but also crashes immediately|
|`wh2`|World Heroes 2|**Works**|
|`wjammers`|Windjammers|Broken sound|
|`zedblade`|Zed Blade|**Works**|

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


