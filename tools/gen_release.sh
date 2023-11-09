#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

TOOLS_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MOBILE_REPO_DIR="$(dirname "${TOOLS_DIR}")"

REPO_NAME="mobilesmartstore"

RELEASE_REPO_URL="$(printf "%s" "YOUR_GIT_PATH")"

cd "${MOBILE_REPO_DIR}"

bloom-release --non-interactive --no-pull-request \
  --override-release-repository-url "${RELEASE_REPO_URL}" \
  --override-release-repository-push-url "${RELEASE_REPO_URL}" \
  --rosdistro "${ROS_DISTRO}" --track "${ROS_DISTRO}" "${REPO_NAME}"
