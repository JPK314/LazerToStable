from typing import Dict, List
import re


class OsuFile:
    def __init__(self, version: str, sections: Dict[str, List[str]]):
        self.version = version
        self.sections = sections

    def to_string(self) -> str:
        result = "osu file format v" + self.version + "\n\n"

        for section, lines in self.sections.items():
            result += "[" + section + "]\n"
            result += "\n".join(lines) + "\n\n"

        return result


def parse_osu_file_from_string(source: str) -> OsuFile:
    sections: Dict[str, List[str]] = {}
    version = None
    current_part = None

    for line in source.splitlines():
        version_match = re.search('osu file format v(\\d+)\n?$', line)
        if version_match:
            version = version_match.group(1)
            continue

        header_match = re.search('^\[(\\w+)\]\n?$', line)
        if header_match:
            current_part = header_match.group(1)
            continue

        if line.strip() == '' or current_part is None:
            continue

        sections.setdefault(current_part, []).append(line)

    if version is None:
        raise Exception("Could not parse osu format version")

    if len(sections.keys()) == 0:
        raise Exception("Could not parse any sections from the osu format")

    if 'HitObjects' not in sections:
        raise Exception("Missing HitObjects section")

    if 'TimingPoints' not in sections:
        raise Exception("Missing TimingPoints section")

    return OsuFile(version, sections)
