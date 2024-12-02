
cat fileList.txt | while read line
do
	python3 file2array.py -f $line -a -o app_http_files.h -t "const char"
done
