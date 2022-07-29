mkdir ~/.ssh/

if [ "$(grep ssh-agent ~/.bashrc)" = "" -a "$USER" = "ros" ]
then
	echo "eval \"\$(ssh-agent -s)\" > /dev/null" >> ~/.bashrc
	echo "ssh-add -D > /dev/null" >> ~/.bashrc
	echo "ssh-add ~/.ssh/sshkey_orin > /dev/null" >> ~/.bashrc
	eval $(ssh-agent -s)
fi

cd
cd .ssh

ssh-keygen -b 2048 -t rsa -f /tmp/sshkey_orin -q -N ""
ssh-copy-id -i /tmp/sshkey_orin ros@orin

mv /tmp/sshkey_orin ~/.ssh/sshkey_orin
mv /tmp/sshkey_orin.pub ~/.ssh/sshkey_orin.pub

ssh-add -D
ssh-add sshkey_orin

ssh-keyscan orin >> ~/.ssh/known_hosts 
