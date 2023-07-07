#pragma once

template <typename T, int capacity>
class Deque {
private:
    T data[capacity];
    int front_idx, back_idx, size;
public:
    Deque() : front_idx(0), back_idx(0), size(0) {}
    
    bool is_empty() {
        return size == 0;
    }
    
    bool is_full() {
        return size == capacity;
    }
    
    bool push_front(T val) {
        if (is_full()) {
            return false;
        }
        front_idx = (front_idx - 1 + capacity) % capacity;
        data[front_idx] = val;
        size++;
        return true;
    }
    
    bool push_back(T val) {
        if (is_full()) {
            return false;
        }
        data[back_idx] = val;
        back_idx = (back_idx + 1) % capacity;
        size++;
        return true;
    }
    
    bool pop_front() {
        if (is_empty()) {
            return false;
        }
        // val = data[front_idx];
        front_idx = (front_idx + 1) % capacity;
        size--;
        return true;
    }
    
    bool pop_back() {
        if (is_empty()) {
            return false;
        }
        back_idx = (back_idx - 1 + capacity) % capacity;
        // val = data[back_idx];
        size--;
        return true;
    }
    
    bool peek_front(T& val) {
        if (is_empty()) {
            return false;
        }
        val = data[front_idx];
        return true;
    }
    
    bool peek_back(T& val) {
        if (is_empty()) {
            return false;
        }
        int idx = (back_idx - 1 + capacity) % capacity;
        val = data[idx];
        return true;
    }
    
    int get_size() {
        return size;
    }
    
    int get_capacity() {
        return capacity;
    }

    T* get_container(){
        return data;
    }
};
