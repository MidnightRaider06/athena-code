// Copyright (c) 2025, UMDLoop
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

#ifndef GENERAL_CONTROLLERS__VISIBILITY_CONTROL_H_
#define GENERAL_CONTROLLERS__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GENERAL_CONTROLLERS__VISIBILITY_EXPORT __attribute__((dllexport))
#define GENERAL_CONTROLLERS__VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define GENERAL_CONTROLLERS__VISIBILITY_EXPORT __declspec(dllexport)
#define GENERAL_CONTROLLERS__VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef GENERAL_CONTROLLERS_BUILDING_DLL
#define GENERAL_CONTROLLERS__VISIBILITY_PUBLIC GENERAL_CONTROLLERS__VISIBILITY_EXPORT
#else
#define GENERAL_CONTROLLERS__VISIBILITY_PUBLIC GENERAL_CONTROLLERS__VISIBILITY_IMPORT
#endif
#define GENERAL_CONTROLLERS__VISIBILITY_PUBLIC_TYPE GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
#define GENERAL_CONTROLLERS__VISIBILITY_LOCAL
#else
#define GENERAL_CONTROLLERS__VISIBILITY_EXPORT __attribute__((visibility("default")))
#define GENERAL_CONTROLLERS__VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define GENERAL_CONTROLLERS__VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define GENERAL_CONTROLLERS__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define GENERAL_CONTROLLERS__VISIBILITY_PUBLIC
#define GENERAL_CONTROLLERS__VISIBILITY_LOCAL
#endif
#define GENERAL_CONTROLLERS__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // GENERAL_CONTROLLERS__VISIBILITY_CONTROL_H_
