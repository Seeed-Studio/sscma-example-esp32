import os
import json
import tarfile
import hashlib
import datetime
import argparse

def calculate_sha1(file_path):
    sha1 = hashlib.sha1()
    with open(file_path, "rb") as file:
        while True:
            data = file.read(65536)
            if not data:
                break
            sha1.update(data)
    return sha1.hexdigest()

def main():
    parser = argparse.ArgumentParser(description="Archive and checksum specific files in a build folder.")
    parser.add_argument("build_path", help="Path to the build folder containing target files")
    parser.add_argument("app_name", help="Name of the the app binary file")
    args = parser.parse_args()

    fn_addr_map = {
        "bootloader.bin": 0,
        "partition-table.bin": 32768,
        args.app_name: 65536
    }

    today = datetime.date.today().strftime("%Y.%m.%d")

    output_folder = os.path.join(os.getcwd(), f"release_v{today}")
    os.makedirs(output_folder, exist_ok=True)

    archive_name = f"v{today}_files.tar.gz"
    with tarfile.open(archive_name, "w:gz") as tar:
        release_info = {}
        release_info["version"] = today
        release_info["bins"] = []

        for root, _, files in os.walk(args.build_path):
            for file in files:
                if file in fn_addr_map.keys():
                    file_path = os.path.join(root, file)
                    arc_name = os.path.relpath(file_path, args.build_path)
                    release_info["bins"].append({
                        "name": file,
                        "url": arc_name,
                        "address": fn_addr_map[file],
                        "checksum": f"SHA-1:{calculate_sha1(file_path)}",
                        "size": os.path.getsize(file_path)
                    })
                    tar.add(file_path, arcname=arc_name)

        info_name = f"v{today}_info.json"
        info_path = os.path.join(output_folder, info_name)
        with open(info_path, "w", encoding="utf-8") as fp:
            json.dump(release_info, fp, indent=4, sort_keys=False)
        tar.add(info_path, arcname=os.path.relpath(info_path, output_folder))
        os.remove(info_path)

    archive_sha1 = calculate_sha1(archive_name)

    new_archive_name = f"v{today}_{archive_sha1}.tar.gz"
    os.rename(archive_name, new_archive_name)

    os.rename(new_archive_name, os.path.join(output_folder, new_archive_name))

    print(f"Archive {new_archive_name} and its SHA-1 checksum stored in folder: {output_folder}")

if __name__ == "__main__":
    main()
