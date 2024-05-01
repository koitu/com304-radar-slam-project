capture_file = "data"

-- TODO: edit this path!!
SAVE_DATA_PATH = "C:\\Users\\ubci\\OneDrive\\Bureau\\COM304\\comm-proj-radar\\record\\new_data\\2\\" .. capture_file .. ".bin"

function get_timestamp()
    local t = os.date("*t")
    local milliseconds = (os.clock() % 1) * 1000
    return string.format("%04d-%02d-%02d %02d:%02d:%02d:%03d", t.year, t.month, t.day, t.hour, t.min, t.sec, milliseconds)
end

local start_timestamp = get_timestamp()
ar1.CaptureCardConfig_StartRecord(SAVE_DATA_PATH, 1)
ar1.StartFrame()
print("Recording started at: " .. start_timestamp)