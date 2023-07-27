#include "espnow_multi.hpp"

#include <utility>

namespace espnow_multi {

namespace {

constexpr auto SEND_FINISHED_BIT = BIT0;

void registerPeer(const uint8_t* peer_addr, uint8_t channel, wifi_interface_t ifidx) {
    esp_now_peer_info_t peer_info;
    peer_info.channel = channel;
    peer_info.ifidx = ifidx;
    peer_info.encrypt = false;

    std::copy(peer_addr, peer_addr + ESP_NOW_ETH_ALEN, peer_info.peer_addr);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
}

void unregisterPeer(const uint8_t* peer_addr) { ESP_ERROR_CHECK(esp_now_del_peer(peer_addr)); }

}  // namespace

EspnowMulti::EspnowMulti() {
    // we can send directly from the start
    xEventGroupSetBits(send_event_group_, SEND_FINISHED_BIT);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));
}

EspnowMulti::~EspnowMulti() {
    ESP_ERROR_CHECK(esp_now_unregister_send_cb());
    ESP_ERROR_CHECK(esp_now_unregister_recv_cb());
    ESP_ERROR_CHECK(esp_now_deinit());
}

std::shared_ptr<EspnowMulti> EspnowMulti::getInstance() {
    // current instance stored in a weak_ptr so that there is at least 0 and at most 1 instance
    static std::weak_ptr<EspnowMulti> instance;

    xSemaphoreTake(accessMutex(), portMAX_DELAY);
    auto shared = instance.lock();
    if (!shared) {
        shared = std::make_shared<EspnowMulti>();
        instance = shared;
    }
    xSemaphoreGive(accessMutex());

    return shared;
}

void EspnowMulti::addReceiver(std::weak_ptr<EspnowReceiver> receiver) {
    xSemaphoreTake(accessMutex(), portMAX_DELAY);
    receivers_.emplace_back(std::move(receiver));
    xSemaphoreGive(accessMutex());
}

esp_err_t EspnowMulti::send(std::weak_ptr<EspnowSender> sender, const uint8_t* peer_addr, const uint8_t* data,
                            size_t len, uint8_t channel, wifi_interface_t ifidx) {
    // we want to wait indefinitely until the last data finished sending
    // need to sync with mutex so that only one thread can send at a time
    xSemaphoreTake(send_mutex_, portMAX_DELAY);
    xEventGroupWaitBits(send_event_group_, SEND_FINISHED_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    xSemaphoreGive(send_mutex_);

    last_sender_ = std::move(sender);
    // register peer before sending
    registerPeer(peer_addr, channel, ifidx);

    esp_err_t ret = esp_now_send(peer_addr, data, len);
    xSemaphoreGive(send_mutex_);

    return ret;
}

SemaphoreHandle_t EspnowMulti::accessMutex() {
    static auto mtx = xSemaphoreCreateMutex();
    return mtx;
}

void EspnowMulti::recv_cb(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) {
    auto multi = EspnowMulti::getInstance();
    // invoke the receive callback for all registered receivers
    // if the receiver expired, remove it from the vector
    for (auto it = multi->receivers_.begin(); it != multi->receivers_.end();) {
        if (auto receiver = it->lock()) {
            receiver->receiveCallback(esp_now_info, data, data_len);
            ++it;
        } else {
            it = multi->receivers_.erase(it);
        }
    }
}

void EspnowMulti::send_cb(const uint8_t* mac_addr, esp_now_send_status_t status) {
    auto multi = EspnowMulti::getInstance();
    // invoke send_cb of the sender
    if (auto sender = multi->last_sender_.lock()) {
        sender->sendCallback(mac_addr, status);
    }
    // can unregister the peer now so that there is space available again
    unregisterPeer(mac_addr);
    // signal that sending is finished
    xEventGroupSetBits(multi->send_event_group_, SEND_FINISHED_BIT);
}

}  // namespace espnow_multi
