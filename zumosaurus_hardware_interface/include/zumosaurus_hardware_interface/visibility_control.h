// Copyright (c) 2022, poine
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ZUMOSAURUS_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define ZUMOSAURUS_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ZUMOSAURUS_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define ZUMOSAURUS_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define ZUMOSAURUS_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define ZUMOSAURUS_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef ZUMOSAURUS_HARDWARE_INTERFACE_BUILDING_DLL
#define ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC ZUMOSAURUS_HARDWARE_INTERFACE_EXPORT
#else
#define ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC ZUMOSAURUS_HARDWARE_INTERFACE_IMPORT
#endif
#define ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC_TYPE ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC
#define ZUMOSAURUS_HARDWARE_INTERFACE_LOCAL
#else
#define ZUMOSAURUS_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define ZUMOSAURUS_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define ZUMOSAURUS_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC
#define ZUMOSAURUS_HARDWARE_INTERFACE_LOCAL
#endif
#define ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // ZUMOSAURUS_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
