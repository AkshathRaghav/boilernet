#!/bin/bash

FILE1="/home/araviki/workbench/boilernet/test/results/scheduler.png"
FILE2="/home/araviki/workbench/boilernet/test/results/scheduler_test.png"

echo -e "Uploading scheduler.png"
curl -X POST "http://localhost:8000/job_store" \
    -F "image=@$FILE1" \
    -F "operation=store" \
    -F "job_identifier=job1"
echo -e ""
echo "Uploading scheduler_test.png\n"
curl -X POST "http://localhost:8000/job_store" \
    -F "image=@$FILE2" \
    -F "operation=store" \
    -F "job_identifier=job2"

echo -e ""
echo -e "Polling store for job1 and job2...\n"
for i in {1..2}; do
    echo -e "Poll $i: job1 store"
    curl -s "http://localhost:8000/poll_op/job1?task_id=1&command=store" | jq -c .
    echo -e "Poll $i: job2 store"
    curl -s "http://localhost:8000/poll_op/job2?task_id=2&command=store" | jq -c .
    sleep 1
done
echo -e ""

echo -e "Enqueuing compute for both..."
curl -X POST "http://localhost:8000/enqueue_op/job1?task_id=1&command=compute"
echo -e ""
curl -X POST "http://localhost:8000/enqueue_op/job2?task_id=2&command=compute"
echo -e ""

sleep 1
echo -e ""

echo -e "Polling compute for job1 and job2..."
for i in {1..2}; do
    echo -e ""
    echo -e "Poll $i: job1 compute"
    curl -s "http://localhost:8000/poll_op/job1?task_id=1&command=compute" | jq -c .
    echo -e "Poll $i: job2 compute"
    curl -s "http://localhost:8000/poll_op/job2?task_id=2&command=compute" | jq -c .
    sleep 1
done
echo -e ""

echo -e "Fetching final images (FETCH)..."
curl -X POST "http://localhost:8000/enqueue_op/job1?task_id=1&command=fetch"
echo -e ""
curl -X POST "http://localhost:8000/enqueue_op/job2?task_id=2&command=fetch"
echo -e ""

sleep 1

for i in {1..2}; do
    echo -e "Poll $i: job1 fetch"
    curl -s "http://localhost:8000/poll_op/job1?task_id=1&command=fetch" | jq -c .
    echo -e "Poll $i: job2 fetch"
    curl -s "http://localhost:8000/poll_op/job2?task_id=2&command=fetch" | jq -c .
    sleep 1
done

echo -e "Test finished!"
