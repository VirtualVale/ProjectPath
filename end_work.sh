#!/bin/bash
echo byebye
time=$(date)
echo "termination time: ${time}"
echo "end ${time}" >> work_times.txt
git add .
read -p "github comment: " comment
git commit -m "${comment}"
git push github master