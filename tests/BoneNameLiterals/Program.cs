// Checks the library's sources (every file of this repository outside tests/, bin/, obj/ and dot-folders):
// - no string literal starting with a bone-name prefix ("j_" or "n_", also $"..." and @"..."): bone names come from the
//   skeleton profile. Comments count too: quote a bone name in a comment without the quotes;
// - the render-object word "DrawObject" nowhere (code, comments, strings);
// - no log prefix of a host's classes ("RagdollController:", "WeaponDropController:"): the library logs "Ragdoll:" and
//   "WeaponDrop:";
// - no bone name (j_... or n_..., quoted or not) inside a string literal, so log texts never name a skeleton's bones;
// - no name from a deny-list anywhere (*.cs, *.md, *.csproj; code, comments and strings): product, company, tool and
//   character-kind names that belong to a host, not to this library. The deny-list is kept as SHA-256 hashes (see
//   DenyList below) so this file does not spell the names out. Whole words, case-insensitive.
// The tests may name bones; they are not scanned.
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;

var root = FindRoot(AppContext.BaseDirectory) ?? FindRoot(Directory.GetCurrentDirectory());
if (root == null)
{
    Console.WriteLine("FAIL: repository root (AdvancedRagdoll.csproj) not found");
    return 2;
}

var literal = new Regex("\"[jn]_", RegexOptions.Compiled);
// Words of a host: its render object and its controllers' log prefixes.
var hostWords = new Regex("DrawObject|RagdollController:|WeaponDropController:", RegexOptions.Compiled);
var stringLiteral = new Regex(@"\$?@?""(?:[^""\\]|\\.)*""", RegexOptions.Compiled);
var boneToken = new Regex(@"(?<![A-Za-z0-9])[jn]_[a-z]", RegexOptions.Compiled);
var denyList = DenyList.Default();

var library = Directory.EnumerateFiles(root, "*", SearchOption.AllDirectories)
    .Select(f => Path.GetRelativePath(root, f).Replace('\\', '/'))
    .Where(r => !r.Split('/').SkipLast(1).Any(d => d is "tests" or "bin" or "obj" or "artifacts" || d.StartsWith('.')))
    .OrderBy(r => r, StringComparer.Ordinal).ToList();
var sources = library.Where(r => r.EndsWith(".cs", StringComparison.Ordinal)).ToList();
// The deny-list check also reads the library's docs and project file.
var libraryAll = library.Where(r => r.EndsWith(".cs", StringComparison.Ordinal) || r.EndsWith(".md", StringComparison.Ordinal) ||
                                    r.EndsWith(".csproj", StringComparison.Ordinal)).ToList();

var failures = 0;
var hostFailures = 0;
foreach (var relative in sources)
{
    var lines = File.ReadAllLines(Path.Combine(root, relative));
    for (var i = 0; i < lines.Length; i++)
    {
        if (literal.IsMatch(lines[i]))
        {
            failures++;
            Console.WriteLine($"{relative}:{i + 1}: {lines[i].Trim()}");
        }
        if (hostWords.IsMatch(lines[i]) || BoneNameInString(lines[i]))
        {
            hostFailures++;
            Console.WriteLine($"{relative}:{i + 1}: (host word or bone name in a string) {lines[i].Trim()}");
        }
    }
}

var nameFailures = 0;
foreach (var relative in libraryAll)
{
    var lines = File.ReadAllLines(Path.Combine(root, relative));
    for (var i = 0; i < lines.Length; i++)
        if (denyList.Matches(lines[i]))
        {
            nameFailures++;
            Console.WriteLine($"{relative}:{i + 1}: (deny-listed name) {lines[i].Trim()}");
        }
}

bool BoneNameInString(string line)
{
    foreach (Match m in stringLiteral.Matches(line))
        if (boneToken.IsMatch(m.Value)) return true;
    return false;
}

// Controls: lines each check must flag, and lines it must let through, or a broken pattern would pass.
var literalControls = new[]
{
    ("var pelvis = \"j_pelvis\";", true),
    ("log.Info($\"n_belly follows\");", true),
    ("Find(@\"j_spine_c\")", true),
    ("// the head bone (in a skeleton j_head) follows", false),
    ("var name = \"gen_\" + i;", false),
};
var literalControlFailures = literalControls.Count(c => literal.IsMatch(c.Item1) != c.Item2);
var hostControls = new[]
{
    ("log.Info($\"RagdollController: Activated\");", true),
    ("log.Info(\"WeaponDropController: removed all weapons\");", true),
    ("// moves the corpse's DrawObject", true),
    ("log.Info($\"Ragdoll: n_belly index={i}\");", true),
    ("log.Info($\"[LegDiag] one of j_leg_a/b did not resolve\");", true),
    ("log.Info($\"Ragdoll: belly-follow bone index={i}, head bone index={j}\");", false),
    ("var name = \"gen_\" + i;", false),
    ("// the head bone (in a skeleton j_head) follows", false),
};
var hostControlFailures = hostControls.Count(c => (hostWords.IsMatch(c.Item1) || BoneNameInString(c.Item1)) != c.Item2);

// The deny-list's matching is checked with a stand-in list of plain words, hashed here, that uses every kind of entry
// the real list has: whole words, two words with whitespace between, two words joined by an apostrophe, a word
// followed by '+', and identifier prefixes (no boundary on the right). The real list then has to let through lines
// that only resemble its words.
var standIn = DenyList.FromWords(
    words: ["otter", "walrus", "bluebird", "blue heron", "jack'o", "widget+"],
    prefixes: ["zq_", "qwxyz"]);
var standInControls = new[]
{
    ("// soft tissue (Otter/WALRUS)", true),
    ("// on a full walrus skeleton", true),
    ("/// bones with zq_ prefixes", true),
    ("// the host's is QwxyzHumanoidProfile", true),
    ("// unlike _qwxyz_value", true),
    ("It started as a Blue  Heron plugin", true),
    ("// tuned on Jack'o and others", true),
    ("// Widget+ scaling survives", true),
    ("// the otter's tail", true),
    ("// otters, walruses, blueheron, blue_heron", false),
    ("// jacko, widget + x, widgets+", false),
    ("var x = azq_count + aqwxyz; // walrus_bone", false),
};
var realControls = new[]
{
    ("return true; // the rule is construed as true", false),
    ("var luminance = ivory_count + array_len; // previous_ya", false),
    ("// the extra soft-tissue bones some skeletons add", false),
    ("// a ruler, aura, hurdle", false),
};
var nameControlFailures = standInControls.Count(c => standIn.Matches(c.Item1) != c.Item2) +
                          realControls.Count(c => denyList.Matches(c.Item1) != c.Item2);
if (denyList.Count != DenyList.ExpectedCount)
{
    Console.WriteLine($"FAIL: the deny-list has {denyList.Count} distinct hashes, expected {DenyList.ExpectedCount}");
    return 2;
}

var coreFiles = sources.Count(f => f.StartsWith("Ragdoll.", StringComparison.Ordinal));
Console.WriteLine($"scanned {sources.Count} library source files, of which {coreFiles} ragdoll core");
if (coreFiles == 0)
{
    Console.WriteLine("FAIL: the check did not find the files it is meant to scan");
    return 2;
}
if (libraryAll.Count(f => f.EndsWith(".md", StringComparison.Ordinal)) == 0 ||
    libraryAll.Count(f => f.EndsWith(".csproj", StringComparison.Ordinal)) == 0)
{
    Console.WriteLine("FAIL: the deny-list check did not find the library's README or project file");
    return 2;
}
if (literalControlFailures > 0)
{
    Console.WriteLine($"FAIL: {literalControlFailures} control line(s) of the bone-name literal check answered wrongly");
    return 2;
}
if (nameControlFailures > 0)
{
    Console.WriteLine($"FAIL: {nameControlFailures} control line(s) of the deny-list check answered wrongly");
    return 2;
}
if (hostControlFailures > 0)
{
    Console.WriteLine($"FAIL: {hostControlFailures} control line(s) of the host word check answered wrongly");
    return 2;
}
if (failures > 0 || hostFailures > 0 || nameFailures > 0)
{
    if (nameFailures > 0)
        Console.WriteLine($"FAIL: {nameFailures} library line(s) with a deny-listed name; write it neutrally");
    if (failures > 0)
        Console.WriteLine($"FAIL: {failures} line(s) with a bone-name literal; ask the skeleton profile instead");
    if (hostFailures > 0)
        Console.WriteLine($"FAIL: {hostFailures} library line(s) with \"DrawObject\", a host log prefix or a bone name in a string");
    return 1;
}
Console.WriteLine($"PASS: no bone-name literals; no host words or deny-listed names in the library ({libraryAll.Count} library files)");
return 0;

static string? FindRoot(string start)
{
    for (var dir = new DirectoryInfo(start); dir != null; dir = dir.Parent)
        if (File.Exists(Path.Combine(dir.FullName, "AdvancedRagdoll.csproj")))
            return dir.FullName;
    return null;
}

// Names the library must not contain, as lowercase hex SHA-256 of their UTF-8 lowercase text. A line is lowercased and
// cut into words (runs of letters, digits and '_', like \w). Candidates hashed per line:
// - every word;
// - every pair of neighbouring words with only whitespace between, as "a b" (whitespace collapsed to one space);
// - every pair joined by exactly one apostrophe, as "a'b";
// - every word directly followed by '+', as "a+";
// - at each position whose previous character is not a-z or 0-9, the next 2..8 characters (identifier prefixes,
//   which have no boundary on the right).
// Words and phrases are looked up in Words, prefixes in Prefixes.
sealed class DenyList
{
    public const int ExpectedCount = 28;

    static readonly string[] WordHashes =
    [
        "1b0460762c0b89da3455ebb7f746785149182bd129f46b0a6ab5146720335003",
        "f8ba04a1a7caa00e41c2580e8fcc4881b15ba4bfc1b491f4f77e406386dc5700",
        "73c986197be8086641cdca2b417632f5960328ff4f8285758bb8483bc029da4e",
        "04c8a7801af863fd8ca205a12be1f2380064bf77c0a5dcc012ad6a3472271267",
        "6d0e0685f9024abdd152b070bbe51a946f518a2148d015828dc9efc9c5814730",
        "765ad9f4c7e62e31e7d0cde6f25e71304116c7df6c8bd45206376f825ff8f0fb",
        "0d53820772a05c89e3da9ac37d716007d49e06902bb36821ee1ff79a63533ff5",
        "b3c81c937314914f68b8df63c67a265e6c2cf7eb86b35f8bedc5ce6051900e45",
        "696cc4f69cc421c6031c490add01083e6ba0c1d0939fb2106b4a687d0f17ce0b",
        "c7197865ca12739fa48013adf1f4800423198c71330c19ae91833459eacb736e",
        "631ca15fff506df1f01e4c520cceea6a14f43feaa765a25e7eb453501fb411bf",
        "e7c497b3ce094a8306c767fbc2d68712d9b112522a1bfe0eb0e25c6fb0d76cfe",
        "c51a36118ac8a9e2c4fd4a501f4d08eea962d130305ffe8a8b1052a19fc8d113",
        "f1aa2680e1133a5bba129f82209ad7f59f7b29b18ff39ad82f9b3c1eb90b9d9d",
        "bd87bd4b98873a1f2b4319bf7e79bf2f6714f38ea1e835e7ef498454aab79ee1",
        "00fe83e6bbfd28dbcba5d87329b462eb724120431e72cda872f1e97d5a875fe6",
        "a5aa505698ad818e3fcca14e0a4a40991e8693268e12fc4d7dacdd95931db2e4",
        "c497f389bad6263912ac4fdfb432d400697f995665b7e79d5e7653e2389f92aa",
        "3ca9db32d70843a815b9a9a91d130982a060a8284655a924c3ae373096adba84",
        "25e28f202f0114039376b0e9238f9668ea55a25bd3ea1f7dbbb29271f8496802",
        "d76dedc723f343365b785ad76ff8ec2b3105a21c85a2fd6ade2c06587a186466",
        "579d45e0f5ca37c1a4f632549810d0cd18b65263476bb3821ed3a535625d7d89",
        "3738f9baa6cd8012af244a9e364efd66d989aba9f697ea7f78ff28075b1f3888",
        "3d9cf1b358578788c7b613492c0726b5fb193acaa8e67a29ec6a7c99d8816c41",
        "95eaf36b35549237a15d4245f08de4cf8623b76647c26f833d23c6efee80aa3b",
    ];

    static readonly string[] PrefixHashes =
    [
        "92069ae87d37bd986edc49a8a2ed870939cbe783a662ff14b93cc0a6a529d412",
        "ac06344d3966443979f7ff61af369c3c26c2c7303a4dbee40c7346b9e967a136",
        "30b4ef592be91e79b33603b07607bc0bf2c0f27260586432d7c36a60634d9ef5",
    ];

    const int MinPrefix = 2, MaxPrefix = 8;
    static readonly Regex Word = new(@"\w+", RegexOptions.Compiled);

    readonly HashSet<string> words;
    readonly HashSet<string> prefixes;

    DenyList(IEnumerable<string> wordHashes, IEnumerable<string> prefixHashes)
    {
        words = new HashSet<string>(wordHashes, StringComparer.Ordinal);
        prefixes = new HashSet<string>(prefixHashes, StringComparer.Ordinal);
    }

    public int Count => words.Count + prefixes.Count;

    public static DenyList Default() => new(WordHashes, PrefixHashes);

    public static DenyList FromWords(IEnumerable<string> words, IEnumerable<string> prefixes) =>
        new(words.Select(Hash), prefixes.Select(Hash));

    public bool Matches(string line)
    {
        var text = line.ToLowerInvariant();
        var tokens = Word.Matches(text);
        for (var t = 0; t < tokens.Count; t++)
        {
            var token = tokens[t];
            if (words.Contains(Hash(token.Value))) return true;
            var end = token.Index + token.Length;
            if (end < text.Length && text[end] == '+' && words.Contains(Hash(token.Value + "+"))) return true;
            if (t + 1 < tokens.Count)
            {
                var next = tokens[t + 1];
                var gap = text.AsSpan(end, next.Index - end);
                if (gap.Length > 0 && gap.Trim().Length == 0 && words.Contains(Hash(token.Value + " " + next.Value)))
                    return true;
                if (gap is "'" && words.Contains(Hash(token.Value + "'" + next.Value))) return true;
            }
        }
        for (var i = 0; i < text.Length; i++)
        {
            if (i > 0 && (text[i - 1] is >= 'a' and <= 'z' or >= '0' and <= '9')) continue;
            for (var n = MinPrefix; n <= MaxPrefix && i + n <= text.Length; n++)
                if (prefixes.Contains(Hash(text.Substring(i, n)))) return true;
        }
        return false;
    }

    static string Hash(string candidate) =>
        Convert.ToHexStringLower(SHA256.HashData(Encoding.UTF8.GetBytes(candidate)));
}
