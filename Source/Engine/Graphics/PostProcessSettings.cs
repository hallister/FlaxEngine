// Copyright (c) Wojciech Figat. All rights reserved.

using System;

namespace FlaxEngine
{
    internal class PostProcessSettingAttribute : Attribute
    {
        public int Bit;

        public PostProcessSettingAttribute(int bit)
        {
            Bit = bit;
        }
    }

    public partial struct AntiAliasingSettings
    {
        /// <summary>
        /// Whether or not to show the TAA settings.
        /// </summary>
        public bool ShowTAASettings => (Mode == AntialiasingMode.TemporalAntialiasing);
    }

    public partial struct ToneMappingSettings
    {
        /// <summary>
        /// Whether or not to show the AgX look.
        /// </summary>
        public bool ShowAgXLook => (Mode == ToneMappingMode.AGX);
    }
}
