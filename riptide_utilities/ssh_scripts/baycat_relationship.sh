mkdir ~/.ssh/

if [ "$(grep ssh-agent ~/.bashrc)" = "" -a "$USER" = "ros" ]
then
	echo "eval \"\$(ssh-agent -s)\" > /dev/null" >> ~/.bashrc
	echo "ssh-add -D > /dev/null" >> ~/.bashrc
	echo "ssh-add ~/.ssh/sshkey_baycat > /dev/null" >> ~/.bashrc
	eval $(ssh-agent -s)
fi

cd
cd .ssh

ssh-keygen -b 2048 -t rsa -f /tmp/sshkey_baycat -q -N ""
ssh-copy-id -i /tmp/sshkey_baycat ros@baycat

mv /tmp/sshkey_baycat ~/.ssh/sshkey_baycat
mv /tmp/sshkey_baycat.pub ~/.ssh/sshkey_baycat.pub

ssh-add -D
ssh-add sshkey_baycat

ssh-keyscan baycat >> ~/.ssh/known_hosts 
