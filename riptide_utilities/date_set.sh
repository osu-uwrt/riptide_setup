date_seconds=$(date +"%s")
echo "$date_seconds"
date_string=$(date --date=@$date_seconds)
sudo date --set="$date_string"
