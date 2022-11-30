# Lazer To Stable

Convert osu!lazer beatmaps to a format compatible with legacy version of the game (v128 -> v14).

## Requirements

- Python 3 (tested with v3.10.6)
- numpy

## Usage

0. If you don't have numpy installed, run this command:
   ```commandline
   pip install numpy
   ```
1. Clone the repository.
   ```commandline
   git clone https://github.com/JPK314/LazerToStable.git
   ```
2. Run the script
   ```commandline
   cd LazerToStable
   python convert.py <path to file>
   ```

The file may be either a plaintext .osu beatmap file or a .osz mapset archive. For .osz archives - all the .osu files inside will be converted and the other files are copied verbatim.

The original file will be copied as a backup and replaced with the converted version.
