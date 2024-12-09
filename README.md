# Qt 期末報告(自動化澆水系統)

## 組員&分工

| 學號  | 姓名 | **工作內容** |
| :--: | :--: | :--: |
| 41243116  | 余家豪 | **ESP32程式撰寫 & 硬體架設** |
| 41243144  | 温博鈞 | **MQTT傳輸 & 硬體傳輸串接** |
| 41243146  | 黃境安 | **GUI介面** |



## 計畫書

##### 使用者利用GUI控制澆水時間以及植物燈開關，利用MQTT將控制訊號傳給ESP32，GUI反饋澆水時間以及環境資訊。

## 傳輸介面(Qt傳給Esp32)

##### `getCurrentData` : 獲取當前澆水時間及植物燈設定時間，回傳Json格式。

| 回傳  | **內容** |
| :--: | :--: |
| id | **設備id** |
| command | **指令** |
| wateringTime | **澆水時間** |
| lightStart | **植物燈開啟時間** |
| lightEnd | **植物燈關閉時間** |

回傳範例:
```json
{
    "id" : "test-0001",
    "command" : "getCurrentData",
    "wateringTime" : "12:00",
    "lightStart" : "20:00",
    "lightStart" : "08:00"
}
```

---

##### `updateData` : 更新澆水及植物燈開關時間
| 回傳  | **內容** |
| :--: | :--: |
| id | **設備id** |
| command | **指令** |
| status | **狀態** |
| error | **錯誤訊息** |

回傳範例:
```json
{
    "id" : "test-0001",
    "command" : "updateData",
    "status" : "200",
    "error" : ""
}
```

---
##### `realTimeIrrigation` : 發送及時澆水訊息
| 回傳  | **內容** |
| :--: | :--: |
| id | **設備id** |
| command | **指令** |
| status | **狀態** |
| error | **錯誤訊息** |

回傳範例:
```json
{
    "id" : "test-0001",
    "command" : "realTimeIrrigation",
    "status" : "500",
    "error" : "The command is currently being implemented"
}
```

---


## 完成進度

- [ ] GUI介面
- [ ] MQTT傳輸
- [ ] Esp32控制
- [ ] 硬體設備
