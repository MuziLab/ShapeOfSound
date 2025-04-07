# Install script for directory: C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/course-project")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/file_for_app/Espressif/tools/xtensa-esp32-elf/esp-12.2.0_20230208/xtensa-esp32-elf/bin/xtensa-esp32-elf-objdump.exe")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mbedtls" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/aes.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/aria.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/asn1.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/asn1write.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/base64.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/bignum.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/block_cipher.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/build_info.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/camellia.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ccm.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/chacha20.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/chachapoly.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/check_config.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/cipher.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/cmac.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/compat-2.x.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/config_adjust_legacy_crypto.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/config_adjust_legacy_from_psa.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/config_adjust_psa_from_legacy.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/config_adjust_psa_superset_legacy.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/config_adjust_ssl.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/config_adjust_x509.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/config_psa.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/constant_time.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ctr_drbg.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/debug.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/des.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/dhm.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ecdh.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ecdsa.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ecjpake.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ecp.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/entropy.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/error.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/gcm.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/hkdf.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/hmac_drbg.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/lms.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/mbedtls_config.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/md.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/md5.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/memory_buffer_alloc.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/net_sockets.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/nist_kw.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/oid.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/pem.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/pk.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/pkcs12.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/pkcs5.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/pkcs7.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/platform.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/platform_time.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/platform_util.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/poly1305.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/private_access.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/psa_util.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ripemd160.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/rsa.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/sha1.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/sha256.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/sha3.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/sha512.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ssl.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ssl_cache.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ssl_ciphersuites.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ssl_cookie.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/ssl_ticket.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/threading.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/timing.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/version.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/x509.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/x509_crl.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/x509_crt.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/mbedtls/x509_csr.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/psa" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/build_info.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_adjust_auto_enabled.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_adjust_config_dependencies.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_adjust_config_key_pair_types.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_adjust_config_synonyms.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_builtin_composites.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_builtin_key_derivation.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_builtin_primitives.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_compat.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_config.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_driver_common.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_composites.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_key_derivation.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_primitives.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_extra.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_legacy.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_platform.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_se_driver.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_sizes.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_struct.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_types.h"
    "C:/file_for_app/Espressif/frameworks/esp-idf-v5.1.6/components/mbedtls/mbedtls/include/psa/crypto_values.h"
    )
endif()

