{
  pkgs ? import <nixpkgs> {}
}:

pkgs.mkShell {
  packages = with pkgs; [
    gcc-arm-embedded
    tio
    openocd
    inetutils
  ];
}
