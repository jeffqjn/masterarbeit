cppname=$1
subname=${cppname##*/}
outname=${subname%.*}
Scale_factor=0.005
Scale_factor_prev=0
Bias=0.01
Bias_prev=0
Failed=1
Faktor_scale=0.5
bias_scale=0.5
start_time=1557924740.1
end_time=1557924860.0
iterationcycle=20
headerfile=/home/noah/masterarbeit/src/angular_v/include/angular_v.h
resultfile=~/masterarbeit/src/angular_v/Erg.txt

function rand(){
min=$1
max=$2
num=$(bc <<< "scale=9; $RANDOM/3640")
echo $num
}
rnd1=$(bc <<< "scale=3;${start_time}+$( rand )")
last_time=${rnd1}
rnd2=$(bc <<< "scale=3;${last_time}+$( rand )")
last_time=${rnd2}
rnd3=$(bc <<< "scale=3;${last_time}+$( rand )")
last_time=${rnd3}
rnd4=$(bc <<< "scale=3;${last_time}+$( rand )")
last_time=${rnd4}
rnd5=$(bc <<< "scale=3;${last_time}+$( rand )")
last_time=${rnd5}
rnd6=$(bc <<< "scale=3;${last_time}+$( rand )")
last_time=${rnd6}
rnd7=$(bc <<< "scale=3;${last_time}+$( rand )")
last_time=${rnd7}
rnd8=$(bc <<< "scale=3;${last_time}+$( rand )")
last_time=${rnd8}
rnd9=$(bc <<< "scale=3;${last_time}+$( rand )")
last_time=${rnd9}
rnd10=$(bc <<< "scale=3;${last_time}+$( rand )")

echo $rnd1
echo $rnd2
echo $rnd3
echo $rnd4
echo $rnd5
echo $rnd6
echo $rnd7
echo $rnd8
echo $rnd9
echo $rnd10
rm $resultfile
	for ((i=1;i<=$iterationcycle;i++));
	do
	

	#apply time

	sed -i "s/ ${start_time}/ ${rnd1}/" ${headerfile}
	sed -i "s/ ${end_time}/ ${rnd2}/" ${headerfile}
	#excute the cpp file
	cd ~/masterarbeit/src/angular_v/build
	make
	cd devel/lib/angular_v/
	echo "here "${outname}
	./${outname}>>${resultfile}
	echo =============="First Test Finished"================
	start_time=$rnd1
	end_time=$rnd2
	
	sed -i "s/ ${start_time}/ ${rnd1}/" ${headerfile}
	sed -i "s/ ${end_time}/ ${rnd3}/" ${headerfile}
	#excute the cpp file
	cd ~/masterarbeit/src/angular_v/build
	make
	cd devel/lib/angular_v/
	echo "here "${outname}
	./${outname}>>${resultfile}
echo =============="Second Test Finished"================
	start_time=$rnd1
	end_time=$rnd3
	
	sed -i "s/ ${start_time}/ ${rnd1}/" ${headerfile}
	sed -i "s/ ${end_time}/ ${rnd4}/" ${headerfile}
	#excute the cpp file
	cd ~/masterarbeit/src/angular_v/build
	make
	cd devel/lib/angular_v/
	echo "here "${outname}
	./${outname}>>${resultfile}
	echo =============="Third Test Finished"================
	start_time=$rnd1
	end_time=$rnd4
	
	sed -i "s/ ${start_time}/ ${rnd1}/" ${headerfile}
	sed -i "s/ ${end_time}/ ${rnd5}/" ${headerfile}
	#excute the cpp file
	cd ~/masterarbeit/src/angular_v/build
	make
	cd devel/lib/angular_v/
	echo "here "${outname}
	./${outname}>>${resultfile}
	echo =============="Fourth Test Finished"================
	start_time=$rnd1
	end_time=$rnd5
	
	sed -i "s/ ${start_time}/ ${rnd1}/" ${headerfile}
	sed -i "s/ ${end_time}/ ${rnd6}/" ${headerfile}
	#excute the cpp file
	cd ~/masterarbeit/src/angular_v/build
	make
	cd devel/lib/angular_v/
	echo "here "${outname}
	./${outname}>>${resultfile}
	echo =============="Fifth Test Finished"================
	start_time=$rnd1
	end_time=$rnd6
	count=0
	while read line
	do
	Testresult=$line
	if [ ${Testresult:0:1} -eq 1 ]
	then
	count=$(bc <<< "$count+1")
	fi
	done < ${resultfile}
	echo "Count is " $count
	if [ ${count} -eq 5 ]
	then
	# shrink the scale with original scale
	echo =====================================
	echo "Test Successful!!"
	echo "Current Scale factor :" ${Scale_factor}
	echo "Current Bias: " ${Bias}
	echo "Current sf Scale: " ${Faktor_scale} 
	echo "Current bias scale " ${bias_scale}
	echo "Begin shrinking the range"
	echo =====================================
	Scale_factor_prev=${Scale_factor}
	Bias_prev=${Bias}

	Scale_factor=$(bc <<< "scale=20;$Scale_factor*$Faktor_scale")
	Bias=$(bc <<< "scale=20;$Bias*$bias_scale")
	# update hedaer file
	sed -i "s/ ${Scale_factor_prev}/ ${Scale_factor}/" ${headerfile}
	sed -i "s/ ${Bias_prev}/ ${Bias}/" ${headerfile}
	# else go back and update the scale
	else
	echo =====================================
	echo "Test Failed"
	echo "Current Scale factor :" ${Scale_factor}
	echo "Current Bias: " ${Bias}
	echo "Current sf Scale: " ${Faktor_scale} 
	echo "Current bias scale " ${bias_scale}
	echo "Change current scale"
	echo =====================================
	for ((i=1;i<=${Failed};i++));
	do 
	
	Faktor_scale=$(bc <<< "scale=20;${Faktor_scale}*0.5")
	bias_scale=$(bc <<< "scale=20;${bias_scale}*0.5")
	done
	Failed=${Failed}+1
	
	Faktor_scale=$(bc <<< "scale=20;1-${Faktor_scale}")
	bias_scale=$(bc <<< "scale=20;1-${bias_scale}")
	Scale_factor_old=${Scale_factor}
	Bias_old=${Bias}

	Scale_factor=$(bc <<< "scale=20;$Scale_factor_prev*$Faktor_scale")
	Bias=$(bc <<< "scale=20;$Bias_prev*$bias_scale")
	# update hedaer file
	sed -i "s/ ${Scale_factor_old}/ ${Scale_factor}/" ${headerfile}
	sed -i "s/ ${Bias_old}/ ${Bias}/" ${headerfile}
	fi
	rm $resultfile
	done




