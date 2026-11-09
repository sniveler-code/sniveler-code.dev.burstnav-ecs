namespace BurstNav
{
    public enum BnTier : byte
    {
        Free = 0,

        Pro = 1,
    }

    public static class BnTierInfo
    {
        public const BnTier Current = BnTier.Free;

        public const bool IsPro = false;

        public const string Name = "Free";
    }
}
