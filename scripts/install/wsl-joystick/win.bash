# https://github.com/MacRover/maxwell/wiki/WSL2-Xbox-Controllers
winget install usbipd --accept-package-agreements

git clone https://github.com/chosterto/WSL-CONFIGS.git c:\\Users\\noskn\\wsl-configs

echo "
[wsl2]
kernel=c:\\Users\\noskn\\wsl-configs\\kernel-xpad\\bzImage
" > c:\\Users\\noskn\\.wslconfig

wsl --shutdown