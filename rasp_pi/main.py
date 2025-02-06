from git_info import get_git_info

__version__ = '0.1.0'

def main():
    print('Hello, World!\tVersion:', __version__, "\tGit: " ,", ".join(get_git_info()))

if __name__ == '__main__':
    main()