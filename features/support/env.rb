require 'rspec/expectations'


DEFAULT_PORT = 5000
DEFAULT_TIMEOUT = 2
ROOT_FOLDER = Dir.pwd
OSM_USER = 'osrm'
OSM_GENERATOR = 'osrm-test'
OSM_UID = 1
TEST_FOLDER = File.join ROOT_FOLDER, 'test'
DATA_FOLDER = 'cache'
OSM_TIMESTAMP = '2000-01-01T00:00:00Z'
DEFAULT_SPEEDPROFILE = 'bicycle'
WAY_SPACING = 100
DEFAULT_GRID_SIZE = 100   #meters
PROFILES_PATH = File.join ROOT_FOLDER, 'profiles'
FIXTURES_PATH = File.join ROOT_FOLDER, 'unit_tests/fixtures'
BIN_PATH = File.join ROOT_FOLDER, 'build'
DEFAULT_INPUT_FORMAT = 'osm'
DEFAULT_ORIGIN = [1,1]
LAUNCH_TIMEOUT = 1
SHUTDOWN_TIMEOUT = 10
DEFAULT_LOAD_METHOD = 'datastore'
OSRM_ROUTED_LOG_FILE = 'osrm-routed.log'

# OS X shim to ensure shared libraries from custom locations can be loaded
# This is needed in OS X >= 10.11 because DYLD_LIBRARY_PATH is blocked
# https://forums.developer.apple.com/thread/9233
if ENV['OSRM_SHARED_LIBRARY_PATH']
  LOAD_LIBRARIES="DYLD_LIBRARY_PATH=#{ENV['OSRM_SHARED_LIBRARY_PATH']} "
else
  LOAD_LIBRARIES=""
end

if ENV['OS']=~/Windows.*/ then
  TERMSIGNAL=9
else
  TERMSIGNAL='TERM'
end


def log_time_and_run cmd
  log_time cmd
  `#{cmd}`
end

def log_time cmd
  puts "[#{Time.now.strftime('%Y-%m-%d %H:%M:%S:%L')}] #{cmd}"
end


puts "Ruby version #{RUBY_VERSION}"
unless RUBY_VERSION.to_f >= 1.9
  raise "*** Please upgrade to Ruby 1.9.x to run the OSRM cucumber tests"
end

if ENV["OSRM_PORT"]
  OSRM_PORT = ENV["OSRM_PORT"].to_i
  puts "Port set to #{OSRM_PORT}"
else
  OSRM_PORT = DEFAULT_PORT
  puts "Using default port #{OSRM_PORT}"
end

if ENV["OSRM_TIMEOUT"]
  OSRM_TIMEOUT = ENV["OSRM_TIMEOUT"].to_i
  puts "Timeout set to #{OSRM_TIMEOUT}"
else
  OSRM_TIMEOUT = DEFAULT_TIMEOUT
  puts "Using default timeout #{OSRM_TIMEOUT}"
end

unless File.exists? TEST_FOLDER
  raise "*** Test folder #{TEST_FOLDER} doesn't exist."
end

def verify_osrm_is_not_running
  if OSRMLoader::OSRMBaseLoader.new.osrm_up?
    raise "*** osrm-routed is already running."
  end
end

def verify_existance_of_binaries
  ["osrm-extract", "osrm-contract", "osrm-routed"].each do |bin|
    unless File.exists? "#{BIN_PATH}/#{bin}#{EXE}"
      raise "*** #{BIN_PATH}/#{bin}#{EXE} is missing. Build failed?"
    end
    unless system "#{LOAD_LIBRARIES}#{BIN_PATH}/#{bin}#{EXE} --help > /dev/null 2>&1"
      log "*** Exited with code #{$?.exitstatus}.", :preprocess
      raise "*** #{LOAD_LIBRARIES}#{BIN_PATH}/#{bin}#{EXE} --help exited with code #{$?.exitstatus}."
    end
  end
end

if ENV['OS']=~/Windows.*/ then
   EXE='.exe'
   QQ='"'
else
   EXE=''
   QQ=''
end

AfterConfiguration do |config|
  clear_log_files
  verify_osrm_is_not_running
  verify_existance_of_binaries
end

at_exit do
  OSRMLoader::OSRMBaseLoader.new.shutdown
end
