#!/bin/bash
for i in {1..100}; \
do time ./detect_road_plane; \
done 2>&1 | grep ^real | sed -e s/.*m// | awk '{sum += $1} END {print sum / NR}'