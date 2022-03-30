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

#ifndef ZUMOSAURUS_CONTROLLERS__VISIBILITY_CONTROL_H_
#define ZUMOSAURUS_CONTROLLERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ZUMOSAURUS_CONTROLLERS_EXPORT __attribute__((dllexport))
#define ZUMOSAURUS_CONTROLLERS_IMPORT __attribute__((dllimport))
#else
#define ZUMOSAURUS_CONTROLLERS_EXPORT __declspec(dllexport)
#define ZUMOSAURUS_CONTROLLERS_IMPORT __declspec(dllimport)
#endif
#ifdef ZUMOSAURUS_CONTROLLERS_BUILDING_DLL
#define ZUMOSAURUS_CONTROLLERS_PUBLIC ZUMOSAURUS_CONTROLLERS_EXPORT
#else
#define ZUMOSAURUS_CONTROLLERS_PUBLIC ZUMOSAURUS_CONTROLLERS_IMPORT
#endif
#define ZUMOSAURUS_CONTROLLERS_PUBLIC_TYPE ZUMOSAURUS_CONTROLLERS_PUBLIC
#define ZUMOSAURUS_CONTROLLERS_LOCAL
#else
#define ZUMOSAURUS_CONTROLLERS_EXPORT __attribute__((visibility("default")))
#define ZUMOSAURUS_CONTROLLERS_IMPORT
#if __GNUC__ >= 4
#define ZUMOSAURUS_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define ZUMOSAURUS_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#else
#define ZUMOSAURUS_CONTROLLERS_PUBLIC
#define ZUMOSAURUS_CONTROLLERS_LOCAL
#endif
#define ZUMOSAURUS_CONTROLLERS_PUBLIC_TYPE
#endif

#endif  // ZUMOSAURUS_CONTROLLERS__VISIBILITY_CONTROL_H_
