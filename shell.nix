# This Nix configuration file defines a build environment for this repository
# for those using the Nix package manager. To use, run `nix-shell`.

{
  pkgs ? import <nixpkgs> {} # TODO: pin version of nixpkgs
}: 

pkgs.mkShell {
  packages = with pkgs; [
    cmake
    meson # for tools
    ninja

    # Device-side
    openocd
    inetutils
    gcc-arm-embedded
    dfu-util # Uploading to hardware through USB
    tio # Viewing serial output
  ];
}
