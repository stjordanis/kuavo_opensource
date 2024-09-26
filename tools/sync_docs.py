import os
import re
import shutil
import sys


def copy_file(src, dest):
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    shutil.copy2(src, dest)
    print(f"Copied {src} to {dest}")


def find_local_images(md_content, md_dir):
    image_links = re.findall(r"!\[.*?\]\((.*?)\)", md_content)
    local_images = []
    for link in image_links:
        if not link.startswith(("http://", "https://")):
            full_path = os.path.abspath(os.path.join(md_dir, link))
            if os.path.exists(full_path):
                local_images.append(full_path)
    return local_images


def should_exclude(file_path, exclude_list):
    file_path = os.path.abspath(file_path)
    file_dir, file_name = os.path.split(file_path)

    for exclude_item in exclude_list:
        if exclude_item.startswith("./"):
            exclude_abs_path = os.path.abspath(exclude_item)
        else:
            exclude_abs_path = os.path.abspath(exclude_item)

        if os.path.isfile(exclude_abs_path):
            if file_path == exclude_abs_path or file_name == os.path.basename(
                exclude_abs_path
            ):
                return True

        elif os.path.isdir(exclude_abs_path):
            if os.path.commonprefix([file_path, exclude_abs_path]) == exclude_abs_path:
                return True

    return False


def should_include(file_path, include_list):
    if not include_list:
        return True

    file_path = os.path.abspath(file_path)
    file_dir, file_name = os.path.split(file_path)

    for include_item in include_list:
        if include_item.startswith("./"):
            include_abs_path = os.path.abspath(include_item)
        else:
            include_abs_path = os.path.abspath(include_item)

        if os.path.isfile(include_abs_path):
            if file_path == include_abs_path or file_name == os.path.basename(
                include_abs_path
            ):
                return True

        elif os.path.isdir(include_abs_path):
            if os.path.commonprefix([file_path, include_abs_path]) == include_abs_path:
                return True

    return False


def copy_md_files_and_images(src_dir, dest_dir, exclude_list, include_list):
    current_dir_name = os.path.basename(os.path.abspath(src_dir))
    dest_dir = os.path.join(dest_dir, current_dir_name)

    for root, _, files in os.walk(src_dir):
        for file in files:
            src_file_path = os.path.join(root, file)

            if should_exclude(src_file_path, exclude_list) or not should_include(src_file_path, include_list):
                continue

            if file.endswith(".md"):
                rel_path = os.path.relpath(src_file_path, src_dir)
                dest_file_path = os.path.join(dest_dir, rel_path)

                copy_file(src_file_path, dest_file_path)

                with open(src_file_path, "r", encoding="utf-8") as md_file:
                    md_content = md_file.read()

                local_images = find_local_images(
                    md_content, os.path.dirname(src_file_path)
                )
                for img_path in local_images:
                    if not should_exclude(img_path, exclude_list) and should_include(img_path, include_list):
                        rel_img_path = os.path.relpath(img_path, src_dir)
                        dest_img_path = os.path.join(dest_dir, rel_img_path)
                        copy_file(img_path, dest_img_path)
                    else:
                        print(f"Skipping excluded or not included image: {img_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python sync_docs.py <source_directory> <target_repo_path>")
        sys.exit(1)

    source_directory = sys.argv[1]
    target_repo_path = sys.argv[2]

    exclude_list = []
    include_list = [
      "docs",
    ]

    copy_md_files_and_images(source_directory, target_repo_path, exclude_list, include_list)