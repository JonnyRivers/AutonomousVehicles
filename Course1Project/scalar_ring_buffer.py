#!/usr/bin/env python3

"""
Ring buffer collection specialized for scalar values
"""

class ScalarRingBuffer:
    def __init__(self, capacity):
        if(capacity < 1):
            raise Exception("capacity must be at least 1")

        self._capacity = capacity
        self._size = 0
        self._nextIndex = 0
        self._values = []

    def insert(self, value):
        if(self._size < self._capacity):
            self._size = self._size + 1
            self._nextIndex = self._nextIndex + 1
            if(self._nextIndex == self._capacity):
                self._nextIndex = 0
            self._values.append(value)
        else:
            self._values[self._nextIndex] = value
            self._nextIndex = self._nextIndex + 1
            if(self._nextIndex == self._capacity):
                self._nextIndex = 0
    
    def sum(self):
        sum = 0
        for i in range(self._size):
            sum = sum + self._values[i]
        
        return sum
