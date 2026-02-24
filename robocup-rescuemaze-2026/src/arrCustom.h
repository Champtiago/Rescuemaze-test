template <typename T>
class arrCustom {
private:
    uint8_t i = 0;
    size_t size;
    T* positions;
    T dummy; // safe fallback — never use the value, just prevents NULL crash

public:
    arrCustom(size_t size, T value) : dummy(value) {
        this->size = size;
        if (size <= 0) {
            this->size = 0;
            positions = new T[0];
            return;
        }
        positions = new T[size];
        for (uint8_t i = 0; i < size; i++) {
            positions[i] = value;
        }
    }

    void set(size_t index, T value) {
        if (index == 255) {
            Serial.println("SET: index is kMaxInt (not found), skipping");
            return;
        }
        if (index < size) {
            positions[index] = value;
        } else {
            Serial.print("SET out of bounds — index: "); Serial.print(index);
            Serial.print(" | size: ");               Serial.println(size);
        }
    }

    // Returns 255 (kMaxInt) if not found
    uint8_t getIndex(T value) const {
        for (uint8_t j = 0; j < i; j++) {
            if (positions[j] == value) return j;
        }
        return 255; // kMaxInt = not found
    }

    // Safe getValue: returns dummy reference if index is invalid
    T& getValue(size_t index) {
        if (index == 255) {
            Serial.println("GET: index is kMaxInt — coord not in map!");
            return dummy;
        }
        if (index >= size) {
            Serial.print("GET out of bounds — index: "); Serial.print(index);
            Serial.print(" | size: ");    Serial.print(size);
            Serial.print(" | used: ");   Serial.println(i);
            return dummy;
        }
        return positions[index];
    }

    // Const version
    const T& getValue(size_t index) const {
        if (index == 255 || index >= size) {
            return dummy;
        }
        return positions[index];
    }

    bool isValid(size_t index) const {
        return (index != 255 && index < size);
    }

    size_t getSize() const { return size; }
    uint8_t getUsed() const { return i; }

    void push_back(T position) {
        if (i >= size) {
            T* temp = new T[size + 1];
            for (uint8_t j = 0; j < size; j++) temp[j] = positions[j];
            delete[] positions;
            positions = temp;
            size++;
        }
        positions[i] = position;
        i++;
    }

    void reset() {
        i = 0;
        for (uint8_t j = 0; j < size; j++) positions[j] = T();
    }
};