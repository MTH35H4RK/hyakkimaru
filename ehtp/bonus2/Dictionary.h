// Dictionary.h
#ifndef DICTIONARY_H
#define DICTIONARY_H

#include <Arduino.h>

template <typename TKey, typename TValue>
class Dictionary {
private:
    struct KeyValuePair {
        TKey key;
        TValue value;
        KeyValuePair* next;
    };

    KeyValuePair* head;

    KeyValuePair* findPair(const TKey& key) const {
        KeyValuePair* current = head;
        while (current != nullptr) {
            if (current->key == key) {
                return current;
            }
            current = current->next;
        }
        return nullptr;
    }

public:
    Dictionary() : head(nullptr) {}

    ~Dictionary() {
        clear();
    }

    TValue& operator[](const TKey& key) {
        KeyValuePair* pair = findPair(key);
        if (pair == nullptr) {
            // Create a new key-value pair if it doesn't exist
            KeyValuePair* newPair = new KeyValuePair();
            newPair->key = key;
            newPair->value = TValue();  // Default value
            newPair->next = head;
            head = newPair;
            return newPair->value;
        } else {
            return pair->value;
        }
    }

    bool remove(const TKey& key) {
        KeyValuePair* current = head;
        KeyValuePair* previous = nullptr;

        while (current != nullptr) {
            if (current->key == key) {
                if (previous == nullptr) {
                    head = current->next;
                } else {
                    previous->next = current->next;
                }
                delete current;
                return true;
            }
            previous = current;
            current = current->next;
        }
        return false;
    }

    void clear() {
        KeyValuePair* current = head;
        while (current != nullptr) {
            KeyValuePair* next = current->next;
            delete current;
            current = next;
        }
        head = nullptr;
    }
};

#endif