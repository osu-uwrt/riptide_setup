mkdir ~/.ssh/

if [ "$(grep ssh-agent ~/.bashrc)" = "" -a "$USER" = "ros" ]
then
	echo "eval \"\$(ssh-agent -s)\" > /dev/null" >> ~/.bashrc
	echo "ssh-add -D > /dev/null" >> ~/.bashrc
	echo "ssh-add ~/.ssh/sshkey_jetson > /dev/null" >> ~/.bashrc
	eval $(ssh-agent -s)
fi

cd
cd .ssh

ssh-keygen -b 2048 -t rsa -f /tmp/sshkey_jetson -q -N ""
ssh-copy-id -i /tmp/sshkey_jetson ros@jetson

mv /tmp/sshkey_jetson ~/.ssh/sshkey_jetson
mv /tmp/sshkey_jetson.pub ~/.ssh/sshkey_jetson.pub

ssh-add -D
ssh-add sshkey_jetson

ssh-keyscan jetson >> ~/.ssh/known_hosts
