import subprocess

def get_git_info():
    try:
        # Ottiene il nome del branch corrente
        branch = subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).strip().decode('utf-8')

        # Ottiene l'hash del commit attuale
        commit = subprocess.check_output(['git', 'rev-parse', 'HEAD']).strip().decode('utf-8')

        return branch, commit
    except subprocess.CalledProcessError:
        return None, None