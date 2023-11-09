# Tools Documentation

## Steps to generate Debian packages

Make sure to update the CHANGELOG of all packages in `mobilesmartstore` with the
new version to release. Then tag the new version with the following commands:
```
$ cd ~/catkin_ws/src/mobilesmartstore
$ catkin_prepare_release --version <new-release-semantic-version>
```

Now, generate the release with the following command:
```
$ ./tools/gen_release.sh
```

To generate the Debian packages, run the script `./tools/gen_deb_pkgs.sh` with
the package name and optionally the release version to generate the Debian
package from:
```
$ ./tools/gen_deb_pkgs.sh -p <package-name> -v <version>
```

For example:
```
$ ./tools/gen_deb_pkgs.sh -p container -v 0.0.6
```

If no version is given, the last version found in the release repository is
used.

The generated Debian package should be placed in the `deb_pkgs`
folder.


## How to use the generate Debian packages?

Unfortunately, apt cannot be used to install the generated packages because the
ROS dependencies are not available in apt repositories as the ROS build farm
doesn't build for the noetic/arm32 target.

To install the packages, run the following command:
```
$ cd ~/catkin_ws/src/mobilesmartstore/deb_pkgs
$ sudo dpkg --install --force-depends <deb-pkg-name>.deb
```

Repeat the process above for all the packages to install, and then update ROS
and apt so they're identified by both tools:
```
$ rosdep update
$ apt update
```

To obtain information about the installed packages, you may try with one of the
following commands:
```
$ dpkg -s <pkg-name>
$ apt show <pkg-name>
```

For example:
```
$ apt show ros-noetic-container
```

To remove the installed packages, use the following command for each package:
```
$ sudo dpkg --remove <pkg-name>
```
