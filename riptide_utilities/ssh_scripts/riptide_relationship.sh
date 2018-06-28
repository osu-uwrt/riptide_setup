mkdir ~/.ssh/

if [ "$(grep ssh-agent ~/.bashrc)" = "" -a "$USER" = "ros" ]
then
	echo "eval \"\$(ssh-agent -s)\" > /dev/null" >> ~/.bashrc
	echo "ssh-add -D > /dev/null" >> ~/.bashrc
	echo "ssh-add ~/.ssh/sshkey_riptide > /dev/null" >> ~/.bashrc
	eval $(ssh-agent -s)
fi

cd
cd .ssh

ssh-keygen -b 2048 -t rsa -f /tmp/sshkey_riptide -q -N ""
ssh-copy-id -i /tmp/sshkey_riptide ros@riptide

mv /tmp/sshkey_riptide ~/.ssh/sshkey_riptide
mv /tmp/sshkey_riptide.pub ~/.ssh/sshkey_riptide.pub

ssh-add -D
ssh-add sshkey_riptide

ssh-keyscan riptide >> ~/.ssh/known_hosts 
