#!/usr/bin/env bash

# Terminal bold colors
BBLACK='\033[1;30m'       # Black
BRED='\033[1;31m'         # Red
BGREEN='\033[1;32m'       # Green
BYELLOW='\033[1;33m'      # Yellow
BBLUE='\033[1;34m'        # Blue
BPURPLE='\033[1;35m'      # Purple
BCYAN='\033[1;36m'        # Cyan
BWHITE='\033[1;37m'       # White

# Terminal reset color
COLOR_OFF='\033[0m'       # Text Reset

### START OF CODE GENERATED BY Argbash v2.9.0 one line above ###
# Argbash is a bash code generator used to get arguments parsing right.
# Argbash is FREE SOFTWARE, see https://argbash.io for more info
# Generated online by https://argbash.io/generate


die()
{
	local _ret="${2:-1}"
	test "${_PRINT_HELP:-no}" = yes && print_help >&2
	echo "$1" >&2
	exit "${_ret}"
}


begins_with_short_option()
{
	local first_option all_short_options='pvh'
	first_option="${1:0:1}"
	test "$all_short_options" = "${all_short_options/$first_option/}" && return 1 || return 0
}

# THE DEFAULTS INITIALIZATION - OPTIONALS
_arg_package_name=
_arg_version=


print_help()
{
	printf "%s%s\n" "Script to generate Debian Packages from ROS packages" \
    " released from bloom"
	printf "Usage: %s %s\n" \
    "$0" "[-p|--package-name <arg>] [-v|--version <arg>] [-h|--help]"
	printf '\t%s%s\n\t%s\n' "-p, --package-name: package name to generate Debian" \
    " image from (if no pkg is" " given it will generate all of them)"
	printf "\t%s%s\n\t%s\n" "-v, --version: semantic version to generate Debian" \
    " image from (if no version" " is given the latest packages are generated)"
	printf "\t%s\n" "-h, --help: Prints help"
}


parse_commandline()
{
	while test $# -gt 0
	do
		_key="$1"
		case "$_key" in
			-p|--package-name)
				test $# -lt 2 && die "Missing value for the optional argument '$_key'." 1
				_arg_package_name="$2"
				shift
				;;
			--package-name=*)
				_arg_package_name="${_key##--package-name=}"
				;;
			-p*)
				_arg_package_name="${_key##-p}"
				;;
			-v|--version)
				test $# -lt 2 && die "Missing value for the optional argument '$_key'." 1
				_arg_version="$2"
				shift
				;;
			--version=*)
				_arg_version="${_key##--version=}"
				;;
			-v*)
				_arg_version="${_key##-v}"
				;;
			-h|--help)
				print_help
				exit 0
				;;
			-h*)
				print_help
				exit 0
				;;
			*)
				_PRINT_HELP=yes die "FATAL ERROR: Got an unexpected argument '$1'" 1
				;;
		esac
		shift
	done
}

parse_commandline "$@"

### END OF CODE GENERATED BY Argbash (sortof) ### ])

contains() {
  ITEM_TO_FIND="${1}"
  CONTAIN_LIST="${2}"
  if [[ "${ITEM_TO_FIND}" =~ (^|[[:space:]])"${CONTAIN_LIST}"($|[[:space:]]) ]]
  then
    RET_VAL="true"
  else
    RET_VAL="false"
  fi
}

is_valid_semantic_version() {
  VERSION_TO_CHECK="${1}"

  SEMVER_REGEX="^([0-9]|[1-9][0-9]*)\.([0-9]|[1-9][0-9]*)\.([0-9]|[1-9][0-9]*)$"
  if [[ "${VERSION_TO_CHECK}" =~ ${SEMVER_REGEX} ]]
  then
    RET_VAL="true"
  else
    RET_VAL="false"
  fi
}

generate_bin_pkg() {
  PKG_NAME="${1}"

  # If no version is given, the last version found in the release repository is used.
  if [[ "${PKG_VERSION}" == "" ]]; then
    TAG_PATTERN="$(printf "%s" "debian/ros-${ROS_DISTRO}-${PKG_NAME}"\
      "_${SEMVER_REGEX}-[0-9]+_${LINUX_DISTRO}")"
  else
    TAG_PATTERN="$(printf "%s" "debian/ros-${ROS_DISTRO}-${PKG_NAME}"\
      "_${PKG_VERSION}-[0-9]+_${LINUX_DISTRO}")"
  fi

  GIT_TARGET_TAG="$(git tag | grep -Ex "${TAG_PATTERN}" | tail -n 1)"

  if [[ "${GIT_TARGET_TAG}" == "" ]]; then
    if [[ "${PKG_VERSION}" == "" ]]; then
      echo -e "${BRED}ERROR:${COLOR_OFF} no release tag could be found for" \
      "package ${PKG_NAME}"
    else
      echo -e "${BRED}ERROR:${COLOR_OFF} no release tag could be found for" \
      "package ${PKG_NAME} at version ${PKG_VERSION}"
    fi

    rm -rf "${TEMP_DIR}"
    return 1
  fi

  git checkout "${GIT_TARGET_TAG}" > /dev/null 2>&1

  git log -n 1

  dpkg-buildpackage -us -uc -b -d
  if [ "$?" -ne "0" ]; then
    echo -e "${BRED}ERROR:${COLOR_OFF} buildpackage couldn't compile the" \
    "package ${BWHITE}${PKG_NAME}${COLOR_OFF}"
    rm -rf "${TEMP_DIR}"
    return 1
  fi

  if [[ "${PKG_VERSION}" == "" ]]; then
  DEB_PATTERN="$(printf "%s" "ros-${ROS_DISTRO}-${PKG_NAME}_${SEMVER_REGEX}"\
    "-[0-9]+${LINUX_DISTRO}_${PROC_ARCH}.deb")"
  else
  DEB_PATTERN="$(printf "%s" "ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}"\
    "-[0-9]+${LINUX_DISTRO}_${PROC_ARCH}.deb")"
  fi

  DEB_FILENAME="$(ls -1 "${TEMP_DIR}" | grep -Ex "${DEB_PATTERN}" | tail -n 1)"

  cp "${TEMP_DIR}/${DEB_FILENAME}" "${DEB_PKGS_DIR}"

  sudo dpkg --install --force-depends "${DEB_PKGS_DIR}/${DEB_FILENAME}"
  if [ "$?" -ne "0" ]; then
    echo -e "${BRED}ERROR:${COLOR_OFF} dpkg couldn't install the" \
    "package ${BWHITE}${PKG_NAME}${COLOR_OFF} successfully"
    rm -rf "${TEMP_DIR}"
    return 1
  fi

  return 0
}

main() {
  # Validate version
  if [[ "${_arg_version}" != "" ]]; then
    is_valid_semantic_version "${_arg_version}"
    if [[ "${RET_VAL}" == "false" ]]; then
      echo -e "${BRED}ERROR:${COLOR_OFF} the version ${_arg_version} is" \
        "invalid, please provide a version following the semantic version format."
      exit 1
    fi
  fi

  source /opt/ros/noetic/setup.bash

  LINUX_DISTRO="buster"
  PROC_ARCH="armhf"
  PKG_VERSION="${_arg_version}"

  TOOLS_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  MOBILE_REPO_DIR="$(dirname "${TOOLS_DIR}")"
  DEB_PKGS_DIR="${MOBILE_REPO_DIR}/deb_pkgs"
  TEMP_DIR="$(mktemp -d)"
  SEMVER_REGEX="([0-9]|[1-9][0-9]*)\.([0-9]|[1-9][0-9]*)\.([0-9]|[1-9][0-9]*)"
  VALID_PKG_NAMES="$(printf "%s" "mss-utils mobile-payment-interface container"\
    " mobile-payment")"

  mkdir -p "${DEB_PKGS_DIR}"
  cd "${TEMP_DIR}"

  RELEASE_REPO_URL="$(printf "%s" "https://softwareBrain:DaZnFHXp4p4pZh7Yp3jp"\
    "@bitbucket.org/ErnestoTV/mobilesmartstore-release.git")"

  RELEASE_REPO_NAME="$(basename ${RELEASE_REPO_URL} .git)"

  git clone "${RELEASE_REPO_URL}"

  cd "${RELEASE_REPO_NAME}"

  # If no package is given, it will generate all of them
  if [[ "${_arg_package_name}" == "" ]]; then
    PKG_NAMES=( $VALID_PKG_NAMES )
    for pkg in ${PKG_NAMES[@]}; do
      generate_bin_pkg "$pkg"
      if [ "$?" -ne "0" ]; then exit 1; fi
    done
  else
    contains "${VALID_PKG_NAMES}" "${_arg_package_name}"
    if [[ "${RET_VAL}" == "false" ]]; then
      echo -e "${BRED}ERROR:${COLOR_OFF} the package name ${_arg_package_name} is" \
        "invalid, valid names are: ${VALID_PKG_NAMES}"
      exit 1
    fi
    generate_bin_pkg "${_arg_package_name}"
    if [ "$?" -ne "0" ]; then exit 1; fi
  fi

  echo "${TEMP_DIR}"
  rm -rf "${TEMP_DIR}"
}

main