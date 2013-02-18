/*
 * This file is part of ALVAR, A Library for Virtual and Augmented Reality.
 *
 * Copyright 2007-2012 VTT Technical Research Centre of Finland
 *
 * Contact: VTT Augmented Reality Team <alvar.info@vtt.fi>
 *          <http://www.vtt.fi/multimedia/alvar.html>
 *
 * ALVAR is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ALVAR; if not, see
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>.
 */

#include "ar_track_alvar/Plugin.h"

#include "ar_track_alvar/Plugin_private.h"

namespace alvar {

Plugin::Plugin(const std::string filename)
    : d(new PluginPrivate())
    , mReferenceCount(new int(1))
{
    d->load(filename);
}

Plugin::Plugin(const Plugin &plugin)
    : d(plugin.d)
    , mReferenceCount(plugin.mReferenceCount)
{
    ++*mReferenceCount;
}

Plugin &Plugin::operator=(const Plugin &plugin)
{
    d = plugin.d;
    mReferenceCount = plugin.mReferenceCount;
    ++*mReferenceCount;
    return *this;
}

Plugin::~Plugin()
{
    if (!--*mReferenceCount) {
        d->unload();
        delete d;
        delete mReferenceCount;
    }
}

void *Plugin::resolve(const char *symbol)
{
    return d->resolve(symbol);
}

} // namespace alvar
