#include <Audio.h>

void AudioPlayQueue::stop(void) {
    audio_block_t *blocks[MAX_BUFFERS + 1];
    uint8_t n = 0;

    AudioNoInterrupts();

    if (userblock != nullptr) {
        blocks[n++] = userblock;
        userblock = nullptr;
    }
    uptr = 0;

    uint8_t t = tail;
    while (t != head) {
        if (++t >= max_buffers) t = 0;

        if (queue[t] != nullptr) {
            blocks[n++] = queue[t];
            queue[t] = nullptr;
        }
    }

    tail = head;

    AudioInterrupts();

    while (n > 0) {
        AudioStream::release(blocks[--n]);
    }
}