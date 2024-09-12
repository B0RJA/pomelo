
cat fileList.txt | while read line
do
	python file2array.py -f $line -a -o uPlot.h -t "const char"
done
