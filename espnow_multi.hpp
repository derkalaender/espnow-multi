#pragma once

#include <esp_err.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#include <memory>
#include <vector>

namespace espnow_multi {

class EspnowReceiver;
class EspnowSender;

class EspnowMulti {
   public:
    /**
     * Returns the singleton instance of this class.
     */
    static std::shared_ptr<EspnowMulti> getInstance();

    EspnowMulti();
    ~EspnowMulti();

    EspnowMulti(const EspnowMulti&) = delete;
    EspnowMulti& operator=(const EspnowMulti&) = delete;

    void addReceiver(std::weak_ptr<EspnowReceiver> receiver);

    /**
     * Send data to a peer. The given sender will be called when the data was sent.
     */
    esp_err_t send(std::weak_ptr<EspnowSender> sender, const uint8_t* peer_addr, const uint8_t* data, size_t len,
                   uint8_t channel = 0, wifi_interface_t ifidx = WIFI_IF_STA);

   private:
    /**
     * Needed to make access to this singleton thread-safe.
     * We init this within a static function instead of globally for thread-safety and to guarantee static
     * initialization order.
     */
    static SemaphoreHandle_t accessMutex();

    /**
     * ESP-NOW receive callback.
     */
    static void recv_cb(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len);

    /**
     * ESP-NOW send callback.
     */
    static void send_cb(const uint8_t* mac_addr, esp_now_send_status_t status);

    // mutex to make other threads block until the last data finished sending
    SemaphoreHandle_t send_mutex_{xSemaphoreCreateMutex()};

    // used to signal finished sending
    EventGroupHandle_t send_event_group_{xEventGroupCreate()};

    // the last interface used to send data
    // needed to call correct sendCallback
    // weak_ptr as it may have already been deleted when the send_cb is called
    std::weak_ptr<EspnowSender> last_sender_;

    // all the registered receivers
    std::vector<std::weak_ptr<EspnowReceiver>> receivers_;
};

class EspnowInterface {
   public:
    virtual ~EspnowInterface() = default;

   protected:
    // Used so that as long as there is a shared_ptr to this object, the EspnowMulti instance will not be deleted.
    std::shared_ptr<EspnowMulti> multi_instance_{EspnowMulti::getInstance()};
};

class EspnowReceiver : public virtual EspnowInterface {
   public:
    virtual void receiveCallback(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) = 0;
};

class EspnowSender : public virtual EspnowInterface {
   public:
    virtual void sendCallback(const uint8_t* peer_addr, esp_now_send_status_t status) = 0;
};

}  // namespace espnow_multi