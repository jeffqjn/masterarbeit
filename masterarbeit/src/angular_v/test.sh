headerfile=/home/noah/masterarbeit/src/angular_v/include/angular_v.h
start_time=1557924740.1
last_time=${start_time}
current_time=${start_time}
dt=0.1
end_time=1557924860.0
while [[ "$end_time" != "${current_time}" ]]
do
current_time=$(bc <<< "${current_time}+${dt}")
sed -i "s/${last_time}/${current_time}/" ${headerfile}
last_time=${current_time}
done
