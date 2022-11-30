from copy import deepcopy
from src.osu import parse_osu_file_from_string, OsuFile
import numpy
from src.main import zero_floor, clamp, process_slider
import argparse
import pathlib
import shutil
import zipfile


def main():
    parser = argparse.ArgumentParser(
        description="Convert osu!lazer beatmaps to a format compatible with legacy version of the game.",
        usage="python convert.py <file>",
    )
    parser.add_argument('file', help="The file to convert, .osu or .osz file.")
    args = parser.parse_args()
    file = pathlib.Path(args.file)

    if not file.exists():
        print('File does not exist.')
        return

    if file.suffix.lower() == '.osu':
        convert_single_file(file)
    elif file.suffix.lower() == '.osz':
        convert_package(file)
    else:
        print('can only handle .osu and .osz files.')


def convert_single_file(path):
    original_path = pathlib.PurePath(path)
    backup_path = pathlib.PurePath(path).with_name(
        original_path.name + ".backup")

    file = path.open(encoding='utf8')
    osu_file = parse_osu_file_from_string(file.read())
    file.close()

    shutil.move(str(original_path), str(backup_path))

    converted_osu_file = process_osu_file(osu_file)
    pathlib.Path(original_path).write_text(converted_osu_file.to_string())


def convert_package(path):
    original_path = pathlib.PurePath(path)
    backup_path = pathlib.PurePath(path).with_name(
        original_path.name + ".backup")

    shutil.move(str(original_path), str(backup_path))

    with zipfile.ZipFile(str(backup_path)) as zip_in, zipfile.ZipFile(str(original_path), "w") as zip_out:
        for info in zip_in.infolist():
            with zip_in.open(info) as file:
                if info.filename.lower().endswith(".osu"):
                    osu_file = parse_osu_file_from_string(
                        file.read().decode("utf-8"))
                    converted_osu_file = process_osu_file(osu_file)
                    zip_out.writestr(
                        info.filename, converted_osu_file.to_string())
                else:
                    zip_out.writestr(info.filename, file.read())


def process_osu_file(osu_file: OsuFile) -> OsuFile:
    new_sections = deepcopy(osu_file.sections)
    new_sections['TimingPoints'] = map(
        process_timing_point_line, osu_file.sections['TimingPoints'])
    new_sections['HitObjects'] = map(lambda line: process_hit_object_line(
        line, osu_file.version), osu_file.sections['HitObjects'])
    return OsuFile('14', new_sections)


def process_timing_point_line(line: str) -> str:
    parts = line.split(",")
    return "%d,%s" % (int(numpy.floor(float(parts[0]))), ",".join(parts[1:]))


def process_hit_object_line(line: str, format_version: str) -> str:
    parts = line.split(",")
    object_type = int(parts[3])
    if not (object_type & 2):
        # not a slider, do not convert
        return line

    x = zero_floor(clamp(float(parts[0]), 131072))
    y = zero_floor(clamp(float(parts[1]), 131072))
    timestamp = int(numpy.floor(float(parts[2])))
    hit_sound = clamp(int(parts[4]), 131072)
    path = parts[5]
    repeats = clamp(int(parts[6]), 131072)
    length = None
    if len(parts) > 7:
        length = max((0, clamp(float(parts[7]), 131072)))
        if length == 0:
            length = None
    rest = ",".join(parts[8:])

    # most of the magic happens here
    new_path = process_slider(x, y, path, format_version)
    return ','.join(map(str, [x, y, timestamp, object_type, hit_sound, new_path, repeats, length, rest]))


if __name__ == "__main__":
    main()
