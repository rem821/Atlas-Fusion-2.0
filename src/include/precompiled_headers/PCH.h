/*
 * Copyright 2023 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once

// STD
#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <istream>
#include <utility>
#include <vector>
#include <queue>
#include <deque>
#include <string>
#include <memory>
#include <limits>
#include <algorithm>
#include <functional>
#include <cmath>
#include <map>
#include <list>
#include <any>
#include <variant>

// Logging
#include <logging/Log.h>
#include <fmt/format.h>

// RTL
#include <rtl/Transformation.h>
#include <rtl/Core.h>

// Thread Pool
#include <BS_thread_pool.hpp>

// Misc
#include <Topics.h>
#include <data_loaders/DataLoaderIdentifiers.h>
#include <RecordingConstants.h>
#include <FrameTypes.h>
#include <TFTree.h>
#include <util/Macros.h>
#include <util/CsvReader.h>
#include <util/Timer.h>
#include <IdentifierToFrameConversions.h>

// EntryPoint
#include <EntryPoint.h>