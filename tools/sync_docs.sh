#!/bin/bash
set -e 

SOURCE_DIR="$CI_PROJECT_DIR"
SOURCE_DIR_NAME="${SOURCE_DIR##*/}"
TARGET_REPO="kuavo-wiki-website"
TARGET_REPO_PATH="/Users/carlos/Desktop/kuavo-wiki-website"
TARGET_DOCS_PATH="/Users/carlos/Desktop/kuavo-wiki-website/docs/basic_usage"
TARGET_DOCS_COPY_PATH="$TARGET_DOCS_PATH/$SOURCE_DIR_NAME"

TARGET_BRANCH="${TARGET_BRANCH:-dev}"

echo $SOURCE_DIR
echo $TARGET_REPO_PATH
echo $TARGET_DOCS_PATH
echo $TARGET_DOCS_COPY_PATH
echo "Target branch: $TARGET_BRANCH"

if git diff --name-only $CI_COMMIT_BEFORE_SHA...$CI_COMMIT_SHA | grep -E -q "\.md$"; then
    echo "Relevant files changed. Proceeding with sync."

    if [ ! -d "$TARGET_REPO_PATH" ]; then
        echo "Target repository not found. Cloning..."
        git clone -b $TARGET_BRANCH ssh://git@www.lejuhub.com:10026/carlos/kuavo-wiki-website.git "$TARGET_REPO_PATH"
        cd "$SOURCE_DIR"
    else
        echo "Target repository found. Updating..."
        cd "$TARGET_REPO_PATH"
        echo "Fetching origin $TARGET_BRANCH..."
        git fetch origin $TARGET_BRANCH
        echo "Resetting to origin/$TARGET_BRANCH..."
        git reset --hard origin/$TARGET_BRANCH
        echo "Removing $TARGET_DOCS_COPY_PATH..."
        rm -rf "$TARGET_DOCS_COPY_PATH"
        echo "Copying $TARGET_DOCS_PATH to $TARGET_DOCS_COPY_PATH..."
        cd "$SOURCE_DIR"
    fi

    python3 scripts/sync_docs.py "$SOURCE_DIR" "$TARGET_DOCS_PATH"
    cd "$TARGET_REPO_PATH"
    has_changes=$(git status --porcelain)
    if [ -z "$has_changes" ]; then
        echo "No changes to commit. Skipping git add and commit."
    else
        echo "Committing changes..."
        git add .
        git commit -m "docs: Sync docs files from kuavo_opensource repo"
        git push origin $TARGET_BRANCH
    fi

    echo "Sync completed."
else
    echo "No relevant files changed. Skipping sync."
fi
