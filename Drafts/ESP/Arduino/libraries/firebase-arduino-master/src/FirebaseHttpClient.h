#ifndef FIREBASE_HTTP_CLIENT_H
#define FIREBASE_HTTP_CLIENT_H

#include <string>

#include "Arduino.h"
#include "Stream.h"

struct HttpStatus {
  static const int TEMPORARY_REDIRECT = 307;
};

class FirebaseHttpClient {
 public:
  static FirebaseHttpClient* create();

  virtual void setReuseConnection(bool reuse) = 0;
  virtual void begin(const std::string& url) = 0;
  virtual void begin(const std::string& host, const std::string& path) = 0;

  virtual void end() = 0;

  virtual void addHeader(const std::string& name, const std::string& value) = 0;
  virtual void collectHeaders(const char* header_keys[],
                              const int header_key_count) = 0;
  virtual std::string header(const std::string& name) = 0;

  virtual int sendRequest(const std::string& method, const std::string& data) = 0;

  virtual std::string getString() = 0;

  virtual Stream* getStreamPtr() = 0;

  virtual std::string errorToString(int error_code) = 0;

  virtual bool connected() = 0;

 protected:
  static const uint16_t kFirebasePort = 443;
};


//"5A:80:FB:80:F8:CD:DF:B4:C0:22:15:C6:42:DF:88:9A:F3:C9:39:75"; // 2022-05
//"2A:44:9A:6C:1D:60:C6:D9:99:99:94:B2:FA:FF:10:08:9F:4B:48:2C"; // generated from https://www.grc.com/fingerprints.htm
static const char kFirebaseFingerprint[] = "2A:44:9A:6C:1D:60:C6:D9:99:99:94:B2:FA:FF:10:08:9F:4B:48:2C"; // generated from https://www.grc.com/fingerprints.htm

      
#endif  // FIREBASE_HTTP_CLIENT_H
