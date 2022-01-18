mkdir ~/.ssh/

if [ "$(grep ssh-agent ~/.bashrc)" = "" -a "$USER" = "ros" ]
then
	echo "eval \"\$(ssh-agent -s)\" > /dev/null" >> ~/.bashrc
	echo "ssh-add -D > /dev/null" >> ~/.bashrc
	echo "ssh-add ~/.ssh/sshkey_xavier > /dev/null" >> ~/.bashrc
	eval $(ssh-agent -s)
fi

cd
cd .ssh

ssh-keygen -b 2048 -t rsa -f /tmp/sshkey_xavier -q -N ""
ssh-copy-id -i /tmp/sshkey_xavier ros@xavier

mv /tmp/sshkey_xavier ~/.ssh/sshkey_xavier
mv /tmp/sshkey_xavier.pub ~/.ssh/sshkey_xavier.pub

ssh-add -D
ssh-add sshkey_xavier

ssh-keyscan xavier >> ~/.ssh/known_hosts 
