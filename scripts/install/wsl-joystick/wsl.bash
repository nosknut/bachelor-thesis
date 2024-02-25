# https://github.com/MacRover/maxwell/wiki/WSL2-Xbox-Controllers
sudo apt install linux-tools-virtual hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20

cd /etc/udev/rules.d
sudo touch 71-inputs.rules
sudo bash -c "echo 'SUBSYSTEM==\"input\", MODE=\"0666\"' >> 71-inputs.rules"
sudo udevadm control --reload-rules && sudo udevadm trigger

cd /etc/ && sudo bash -c "echo -e '[boot]\nsystemd=true' >> wsl.conf"

echo "Please run wsl --shutdown from PowerShell or CMD to restart WSL2"
